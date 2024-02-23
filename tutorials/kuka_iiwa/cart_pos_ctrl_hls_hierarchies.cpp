#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity/VelocityScene.hpp>
#include <solvers/hls/HierarchicalLSSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <controllers/JointPosPDController.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;

/**
 * Velocity-based example for a simple task hierarchy: Cartesian position control of the end effector on the highest priority
 * and joint position control of the elbow joint on the lower priority.
 */
int main(int argc, char** argv){

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();

    // Configure the robot model by passing the RobotModelConfig. The simplest configuration can by obtained by only setting
    // the path to the URDF file. In doing so, WBC will assume:
    // - The robot is fixed base
    // - The robot is fully actuated
    // - The order of joints used inside WBC is the same as the one obtained by the URDF parser (which is alphabetic)
    // For all configuration options, check core/RobotModelConfig.hpp
    RobotModelConfig config;
    config.file_or_string = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    if(!robot_model->configure(config))
        return -1;

    // Create a solver. Each solver in WBC is derived from a common Solver class as the solver will later be passed to the WBC scene.
    // Use HierarchicalLSSolver, which is an analitical solver and implements a sequence of nullspace projections to define a
    // strict task hierarchy. For a single task, the solution will degrade to a simple Pseudo Inversion.
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    std::dynamic_pointer_cast<HierarchicalLSSolver>(solver)->setMaxSolverOutputNorm(10); // Solver specific config: Maximum allowed norm of the solution vector

    // Configure the WBC Scene by passing the WBC config. Each entry in this vector corresponds to a task. Multiple tasks can be
    // configured by passing multiple task configurations and assigning them a corresponding priority.
    // Here, we consider two tasks:  joint position control of joint 5 on the highest priority
    // and Cartesian position control of the end effector on the lower priority.

    TaskConfig jnt_task;
    jnt_task.name        = "jnt_pos_ctrl_elbow";   // Unique identifier
    jnt_task.type        = jnt;                    // Cartesian or joint space task?
    jnt_task.priority    = 0;                      // Priority, 0 - highest prio
    jnt_task.joint_names = {"kuka_lbr_l_joint_5"}; // Joint names involved in the task
    jnt_task.activation = 1;                       // (0..1) initial task activation. 1 - Task should be active initially
    jnt_task.weights    = vector<double>(1,1);     // Task weights. Can be used to balance the relativ importance of the task variables

    TaskConfig cart_task;
    cart_task.name       = "cart_pos_ctrl_left"; // Unique identifier
    cart_task.type       = cart;                 // Cartesian or joint space task?
    cart_task.priority   = 1;                    // Priority, 0 - highest prio
    cart_task.root       = "kuka_lbr_l_link_0";  // Root link of the kinematic chain to consider for this task
    cart_task.tip        = "kuka_lbr_l_tcp";     // Tip link of the kinematic chain to consider for this task
    cart_task.ref_frame  = "kuka_lbr_l_link_0";  // In what frame is the task specified?
    cart_task.activation = 1;                    // (0..1) initial task activation. 1 - Task should be active initially
    cart_task.weights    = vector<double>(6,1);  // Task weights. Can be used to balance the relativ importance of the task variables (e.g. position vs. orienration)

    VelocityScene scene(robot_model, solver, 1e-3);

    if(!scene.configure({cart_task, jnt_task}))
        return -1;

    // Configure the controllers, one Cartesian position controller, one joint Position controller
    CartesianPosPDController cart_controller;
    cart_controller.setPGain(base::Vector6d::Constant(1)); // Set kp
    JointPosPDController jnt_controller(jnt_task.joint_names);
    base::VectorXd p_gain(1);
    p_gain.setConstant(1);
    jnt_controller.setPGain(p_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    base::samples::Joints joint_state;
    uint nj = robot_model->noOfJoints();
    joint_state.resize(nj);
    joint_state.names = robot_model->jointNames(); // All joints from the robot
    for(int i = 0; i < nj; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now(); // Set a valid timestamp, otherwise the robot model will throw an error

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    base::samples::RigidBodyStateSE3 setpoint_cart, ctrl_output_cart, feedback_cart;
    base::samples::Joints setpoint_jnt, ctrl_output_jnt, feedback_jnt;
    setpoint_cart.pose.position = base::Vector3d(0.0, 0.0, 0.8);
    setpoint_cart.frame_id = cart_task.ref_frame;
    setpoint_cart.pose.orientation.setIdentity();
    feedback_cart.pose.position.setZero();
    feedback_cart.pose.orientation.setIdentity();
    setpoint_jnt.resize(jnt_task.joint_names.size());
    setpoint_jnt.names = jnt_task.joint_names;
    setpoint_jnt[0].position = 0.1;

    // Run control loop
    double loop_time = 0.01; // seconds
    base::commands::Joints solver_output;
    while((setpoint_cart.pose.position - feedback_cart.pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state);

        // Update cart controller. The feedback is the pose of the tip link described in ref_frame link
        feedback_cart    = robot_model->rigidBodyState(cart_task.ref_frame, cart_task.tip);
        ctrl_output_cart = cart_controller.update(setpoint_cart, feedback_cart);

        // Update jnt controller. The feedback are the joint positions of the related joints
        feedback_jnt    = robot_model->jointState(jnt_task.joint_names);
        ctrl_output_jnt = jnt_controller.update(setpoint_jnt, feedback_jnt);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(cart_task.name, ctrl_output_cart);
        scene.setReference(jnt_task.name, ctrl_output_jnt);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Update the current joint state. Simply integrate the current joint position using the joint velocity given by the solver.
        // On a real robot, this would be replaced by a function that sends the solver output to the joints.
        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += solver_output[i].speed * loop_time;
        joint_state.time = base::Time::now();

        cout<<"setpoint: x:   "<<setpoint_cart.pose.position(0)<<" y: "<<setpoint_cart.pose.position(1)<<" z: "<<setpoint_cart.pose.position(2)<<endl;
        cout<<"setpoint: qx:  "<<setpoint_cart.pose.orientation.x()<<" qy: "<<setpoint_cart.pose.orientation.y()<<" qz: "<<setpoint_cart.pose.orientation.z()<<" qw: "<<setpoint_cart.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:    "<<feedback_cart.pose.position(0)<<" y: "<<feedback_cart.pose.position(1)<<" z: "<<feedback_cart.pose.position(2)<<endl;
        cout<<"feedback qx:   "<<feedback_cart.pose.orientation.x()<<" qy: "<<feedback_cart.pose.orientation.y()<<" qz: "<<feedback_cart.pose.orientation.z()<<" qw: "<<feedback_cart.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Position:      "; for(int i = 0; i < nj; i++) cout<<joint_state[i].position<<" "; cout<<endl;
        cout<<"Velocity:      "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
