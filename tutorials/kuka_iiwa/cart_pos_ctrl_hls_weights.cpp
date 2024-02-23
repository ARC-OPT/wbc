#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity/VelocityScene.hpp>
#include <solvers/hls/HierarchicalLSSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;

/**
 * Same Velocity-based example as in tutorial 01. Only that the tasks weights are modified, so that the solution only considers the
 * task space position, not the orientation.
 */
int main(){

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
    // configured by passing multiple task configurations and assigning them a corresponding priority. Here, we consider only
    // a single Cartesian task. The most important entries here are root/tip and reference frame, any of which has to be a valid
    // link in the URDF model. For all configuration options, check core/TaskConfig.hpp
    TaskConfig cart_task;
    cart_task.name       = "cart_pos_ctrl_left"; // Unique identifier
    cart_task.type       = cart;                 // Cartesian or joint space task?
    cart_task.priority   = 0;                    // Priority, 0 - highest prio
    cart_task.root       = "kuka_lbr_l_link_0";  // Root link of the kinematic chain to consider for this task
    cart_task.tip        = "kuka_lbr_l_tcp";     // Tip link of the kinematic chain to consider for this task
    cart_task.ref_frame  = "kuka_lbr_l_link_0";  // In what frame is the task specified?
    cart_task.activation = 1;                    // (0..1) initial task activation. 1 - Task should be active initially

    // Set the task weights for the orientation dof to zero! As a result, the orientation will be arbitrary, only the position will be tracked!
    cart_task.weights    = {1,1,1,0,0,0};
    VelocityScene scene(robot_model, solver, 1e-3);
    if(!scene.configure({cart_task}))
        return -1;

    // Set the joint weights. Set the weight to the elbow joint to zero! As a result the elbow joint
    // will not contribute to the task solution, i.e., its velocity will be zero!
    JointWeights joint_weights;
    joint_weights.names = robot_model->jointNames();
    joint_weights.elements = {1,1,1,1,0,1,1};
    scene.setJointWeights(joint_weights);

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1)); // Set kp

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    base::samples::Joints joint_state;
    uint nj = robot_model->noOfJoints();
    joint_state.resize(nj);
    joint_state.names = robot_model->jointNames(); // All joints from the robot
    for(uint i = 0; i < nj; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now(); // Set a valid timestamp, otherwise the robot model will throw an error

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    base::samples::RigidBodyStateSE3 setpoint, ctrl_output, feedback;
    setpoint.pose.position = base::Vector3d(0.0, 0.0, 0.8);
    setpoint.frame_id = cart_task.ref_frame;
    setpoint.pose.orientation.setIdentity();
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = 0.01; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        feedback = robot_model->rigidBodyState(cart_task.ref_frame, cart_task.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(cart_task.name, ctrl_output);

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

        cout<<"setpoint: x:   "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx:  "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:    "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx:   "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(uint i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
