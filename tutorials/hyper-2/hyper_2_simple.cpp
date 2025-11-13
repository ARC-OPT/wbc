#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/acceleration_reduced_tsid/AccelerationSceneReducedTSID.hpp>
#include <controllers/JointPosPDController.hpp>
#include <tasks/JointAccelerationTask.hpp>
#include <tools/JointIntegrator.hpp>
#include <unistd.h>
#include <chrono>

using namespace wbc;
using namespace std;
using namespace qpOASES;
using namespace wbc;

int main(){

    double dt = 0.001;

    // Create robot model
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();

    // Configure a serial robot model with floating base and two contact points: {"LLAnkle_FT", "LRAnkle_FT"}.
    // Note that the joint names have to contain {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
    // "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"} in addition to the actuated joints.
    // Also a valid initial state (pose/twist/acceleration) of the floating base should be passed
    types::RigidBodyState floating_base_state;
    floating_base_state.pose.position = Eigen::Vector3d(-0.0248, 0.0, 0.72848);
    floating_base_state.pose.orientation = Eigen::Quaterniond(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config;
    config.file_or_string = "../../../models/hyper-2/urdf/HyPer-2.urdf";
    config.floating_base = true;
    config.contact_points = {types::Contact("link_ll_foot",1,1.0),types::Contact("link_lr_foot",1,1.0)};
    if(!robot_model->configure(config))
        return -1;

    // Configure solver. We have to use QPOases in this case, as it can deal with hard constraints
    // like rigid feet contact points
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    Options options = std::dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene, we have to use VelocitySceneQuadraticCost here, since it implements
    // rigid contact constraints for the feet contacts. Create a task for controlling the root link in
    // world coordinates

    JointAccelerationTaskPtr jnt_task;
    uint nj = robot_model->na();
    jnt_task = make_shared<JointAccelerationTask>(TaskConfig("joint_position",0,vector<double>(nj,1),1   ),
                                                   robot_model,
                                                   robot_model->jointNames());
    AccelerationSceneReducedTSID scene(robot_model, solver, dt);
    if(!scene.configure({jnt_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    JointPosPDController controller(nj);
    Eigen::VectorXd p_gain(nj), d_gain(nj);
    p_gain.setConstant(2.0);
    d_gain.setConstant(0.5);
    controller.setPGain(p_gain);
    controller.setDGain(d_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed.
    // Note that you don't have to pass the floating base pose here.

    types::JointState joint_state;
    joint_state.resize(nj);
    joint_state.position << 0., 1.57,-0.7, 0,0,0,0.2,-0.15,0, 0.,-1.57,0.7, 0,0,0,0.2,-0.15,0;
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::JointState ref_joint_state;
    ref_joint_state.resize(nj);
    Eigen::VectorXd ctrl_output;
    ref_joint_state.position << 0., 1.57,-0.7, 0,0,0,0.2,-0.17,0, 0.,-1.57,0.7, 0,0,0,0.2,-0.15,0;
    ref_joint_state.velocity.setZero();
    ref_joint_state.acceleration.setZero();

    // Run control loop
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    JointIntegrator joint_integrator;
    while((ref_joint_state.position - joint_state.position).norm() > 1e-4){

        auto s = std::chrono::high_resolution_clock::now();

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        // Note that you have to pass the floating base state as well now!
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration,
                            floating_base_state.pose,
                            floating_base_state.twist,
                            floating_base_state.acceleration);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        ctrl_output = controller.update(ref_joint_state.position, ref_joint_state.velocity, ref_joint_state.acceleration,
                                        joint_state.position, joint_state.velocity);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        jnt_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        joint_integrator.integrate(joint_state, solver_output, loop_time, types::CommandMode::ACCELERATION);

        auto e = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(e-s).count();

        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<robot_model->jointNames()[i]<<" "; cout<<endl;
        cout<<"Position:      "; cout<<solver_output.position.transpose()<<" "; cout<<endl;
        cout<<"Velocity:      "; cout<<solver_output.velocity.transpose()<<" "; cout<<endl;
        cout<<"Acceleration:  "; cout<<solver_output.acceleration.transpose()<<" "; cout<<endl;
        cout<<"Effort:        "; cout<<solver_output.effort.transpose()<<" "; cout<<endl;
        cout<<"Ext. Force     "; for(const auto& wrench : scene.getContactWrenches()) cout<<wrench.force.transpose()<<" "; cout<<endl;
        cout<<"Ext. Torque    "; for(const auto& wrench : scene.getContactWrenches()) cout<<wrench.torque.transpose()<<" "; cout<<endl;
        cout<<"solve time:    " << solve_time << " (mu s)" << endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;


        joint_state.position = solver_output.position;
        joint_state.velocity = solver_output.velocity;


        usleep(loop_time * 1e6);
        break;
    }

    return 0;
}
