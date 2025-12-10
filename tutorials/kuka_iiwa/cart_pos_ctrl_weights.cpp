#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>
#include <tasks/SpatialVelocityTask.hpp>

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
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    // Configure the WBC Scene by passing the WBC config. Each entry in this vector corresponds to a task. Multiple tasks can be
    // configured by passing multiple task configurations and assigning them a corresponding priority. Here, we consider only
    // a single Cartesian task. The most important entries here are root/tip and reference frame, any of which has to be a valid
    // link in the URDF model. For all configuration options, check core/TaskConfig.hpp
    SpatialVelocityTaskPtr cart_task;
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(6);
    weights.segment(3,3).setZero(); // Set orientation weights to zero
    cart_task = make_shared<SpatialVelocityTask>(TaskConfig("cart_pos_ctrl",0,weights,1),
                                                   robot_model,
                                                   "kuka_lbr_l_tcp");
    VelocitySceneQP scene(robot_model, solver, 1e-3);
    if(!scene.configure({cart_task}))
        return -1;

    // Set the joint weights. Set the weight to the elbow joint to zero! As a result the elbow joint
    // will not contribute to the task solution, i.e., its velocity will be zero!
    Eigen::VectorXd joint_weights(robot_model->nj());
    joint_weights << 1,1,1,1,1,1,1;
    robot_model->setJointWeights(joint_weights);

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    Eigen::VectorXd p_gain(6);
    p_gain.setConstant(1.0);
    controller.setPGain(p_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    types::JointState joint_state;
    uint nj = robot_model->nj();
    joint_state.resize(nj);
    joint_state.position.setConstant(0.1);

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::Pose ref_pose, pose;
    types::Twist ref_twist, ctrl_output;
    ref_pose.position = Eigen::Vector3d(0.0, 0.0, 0.95);
    ref_pose.orientation.setIdentity();
    pose.position.setZero();
    pose.orientation.setIdentity();
    ref_twist.setZero();

    // Run control loop
    double loop_time = 0.01; // seconds
    types::JointCommand solver_output;
    while((ref_pose.position - pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        pose = robot_model->pose(cart_task->tipFrame());
        ctrl_output = controller.update(ref_pose,ref_twist,pose);

        // Update tasks. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Update the current joint state. Simply integrate the current joint position using the joint velocity given by the solver.
        // On a real robot, this would be replaced by a function that sends the solver output to the joints.
        joint_state.position += solver_output.velocity * loop_time;

        cout<<"setpoint: x:   "<<ref_pose.position(0)<<" y: "<<ref_pose.position(1)<<" z: "<<ref_pose.position(2)<<endl;
        cout<<"setpoint: qx:  "<<ref_pose.orientation.x()<<" qy: "<<ref_pose.orientation.y()<<" qz: "<<ref_pose.orientation.z()<<" qw: "<<ref_pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:    "<<pose.position(0)<<" y: "<<pose.position(1)<<" z: "<<pose.position(2)<<endl;
        cout<<"feedback qx:   "<<pose.orientation.x()<<" qy: "<<pose.orientation.y()<<" qz: "<<pose.orientation.z()<<" qw: "<<pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<robot_model->jointNames()[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; cout<<solver_output.velocity.transpose()<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
