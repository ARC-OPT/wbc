#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/acceleration_reduced_tsid/AccelerationSceneReducedTSID.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/SpatialAccelerationTask.hpp>
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
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();

    // Configure a serial robot model with floating base and two contact points: {"LLAnkle_FT", "LRAnkle_FT"}.
    // Note that the joint names have to contain {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
    // "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"} in addition to the actuated joints.
    // Also a valid initial state (pose/twist/acceleration) of the floating base should be passed
    types::RigidBodyState floating_base_state;
    floating_base_state.pose.position = Eigen::Vector3d(0.0, 0.0, 0.72848);
    floating_base_state.pose.orientation = Eigen::Quaterniond(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config;
    config.file_or_string = "../../../models/hyper-2/submechanisms/HyPer-2_full.urdf";
    config.submechanism_file = "../../../models/hyper-2/submechanisms/submechanisms_floating_base.yml";
    config.floating_base = true;
    config.contact_points = {types::Contact("link_ll_foot", // Contact frame ID
                                             1,             // Contact active?
                                             1.0,           // Coulomb friction coefficient
                                             0.15,         // x-dimension of contact surface
                                             0.22),         // y-dimension of contact surface
                             types::Contact("link_lr_foot",
                                             1,
                                             1.0,
                                             0.15,
                                             0.22)};
    if(!robot_model->configure(config)){
        std::cerr<<"Failed to configure Robot Model."<<std::endl;
        return -1;
    }

    // We need the Fixed base model here to compute the forward system state (not possible for floating base systems)
    hyrodyn::RobotModel_HyRoDyn hyrodyn;
    string urdf_file = "../../../models/hyper-2/submechanisms/HyPer-2_full.urdf";
    string submechanism_file = "../../../models/hyper-2/submechanisms/submechanisms.yml";
    hyrodyn.load_robotmodel(urdf_file, submechanism_file);

    // Configure solver. We have to use QPOases in this case, as it can deal with hard constraints
    // like rigid feet contact points
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    Options options = std::dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene, we have to use VelocitySceneQuadraticCost here, since it implements
    // rigid contact constraints for the feet contacts. Create a task for controlling the root link in
    // world coordinates

    SpatialAccelerationTaskPtr cart_task;
    cart_task = make_shared<SpatialAccelerationTask>(TaskConfig("base_pose",0,Eigen::VectorXd::Ones(6),1),
                                                     robot_model,
                                                     "base_link");
    AccelerationSceneReducedTSID scene(robot_model, solver, dt, 6);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    Eigen::VectorXd p_gain(6),d_gain(6);
    p_gain.segment(0,3).setConstant(3.0);
    p_gain.segment(3,3).setZero();
    d_gain.segment(0,3).setConstant(3.0);
    d_gain.segment(3,3).setZero();
    controller.setPGain(p_gain);
    controller.setDGain(d_gain);
    

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed.
    // Note that you don't have to pass the floating base pose here.

    types::JointState joint_state;
    joint_state.resize(robot_model->na());
    joint_state.position << 0.0,  0.0,-0.3, 0.6,-0.33, 0.0, // Left Leg
                            0.0,  0.0,-0.3, 0.6,-0.33, 0.0, // Right Leg
                            0.0,-1.57, 0.0,-0.7,            // Left Arm
                            0.0, 1.57, 0.0, 0.7;            // Right Arm
                            
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();

    types::JointState actuator_state;
    hyrodyn.y = joint_state.position;
    hyrodyn.yd = joint_state.velocity;
    hyrodyn.ydd = joint_state.acceleration;
    hyrodyn.calculate_system_state();
    actuator_state.position = hyrodyn.u;
    actuator_state.velocity = hyrodyn.ud;
    actuator_state.acceleration = hyrodyn.udd;

    // Choose a valid reference pose x_r, which is defined in robot base frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::Pose ref_pose, pose;
    types::Twist ref_twist, twist;
    types::SpatialAcceleration ctrl_output, ref_acc;
    ref_pose.position = Eigen::Vector3d(0.0,0,0.7);
    ref_pose.orientation = Eigen::Quaterniond(1,0,0,0);
    ref_twist.setZero();
    ref_acc.setZero();

    // Run control loop
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    JointIntegrator joint_integrator;
    while((ref_pose.position - pose.position).norm() > 1e-4){

        auto s = std::chrono::high_resolution_clock::now();

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        // Note that you have to pass the floating base state as well now!
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration,
                            floating_base_state.pose,
                            floating_base_state.twist,
                            floating_base_state.acceleration);

        pose = robot_model->pose(cart_task->tipFrame());
        twist = robot_model->twist(cart_task->tipFrame());

        // Update controller. The feedback is the pose of the tip link described in world frame
        ctrl_output = controller.update(ref_pose, ref_twist, ref_acc, pose, twist);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint acceleration that achieves the task space acceleration demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Integrate actuator acceleration to get actuator velocity and position
        joint_integrator.integrate(actuator_state, solver_output, dt, types::CommandMode::ACCELERATION);

        // Set actuator state equal to solver output for next iteration
        actuator_state.position = solver_output.position;
        actuator_state.velocity = solver_output.velocity;
        actuator_state.acceleration = solver_output.acceleration;

        // Use Hyrodyn to compute the state in independent joint space from the solver output, which only contact
        hyrodyn.u = actuator_state.position;
        hyrodyn.ud = actuator_state.velocity;
        hyrodyn.udd = solver_output.acceleration;        
        hyrodyn.calculate_forward_system_state();

        joint_state.position = hyrodyn.y;
        joint_state.velocity = hyrodyn.yd;
        joint_state.acceleration = hyrodyn.ydd;

        // Update floating base pose, use an over-simplistic estimation
        types::Pose pose_1 = robot_model->pose("base_link");
        types::Pose pose_2 = robot_model->pose("link_ll_foot");
        types::Pose pose_3;
        Eigen::Affine3d t1,t2,t3;
        t1 = pose_1.orientation;
        t1.pretranslate(pose_1.position);
        t2 = pose_2.orientation;
        t2.pretranslate(pose_2.position);
        t3 = t2.inverse()*t1;
        floating_base_state.pose.position = t3.translation();
        floating_base_state.pose.position[0]=floating_base_state.pose.position[1]=0;

        auto e = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(e-s).count();

        cout<<"Solver output: "; cout<<endl;
        cout<<"Ind. Joints:   "; for(uint i = 0; i < hyrodyn.independent_dof; i++) cout<<hyrodyn.jointnames_independent[i]<<" "; cout<<endl;
        cout<<"Act. Joints:   "; for(uint i = 0; i < hyrodyn.active_dof; i++) cout<<hyrodyn.jointnames_active[i]<<" "; cout<<endl;
        cout<<"Acceleration:  "; cout<<solver_output.acceleration.transpose()<<" "; cout<<endl;
        cout<<"Effort:        "; cout<<solver_output.effort.transpose()<<" "; cout<<endl;
        cout<<"Ext. Force     "; for(const auto& wrench : scene.getContactWrenches()) cout<<wrench.force.transpose()<<" "; cout<<endl;
        cout<<"Ext. Torque    "; for(const auto& wrench : scene.getContactWrenches()) cout<<wrench.torque.transpose()<<" "; cout<<endl;
        cout<<"State:         "; cout<<endl;
        cout<<"Ref. Pose:     "; cout<<ref_pose.position.transpose()<<" "<<ref_pose.orientation.coeffs().transpose(); cout<<endl;
        cout<<"Pose:          "; cout<<pose.position.transpose()<<" "<<pose.orientation.coeffs().transpose(); cout<<endl;
        cout<<"Twist:         "; cout<<twist.linear.transpose()<<" "<<twist.angular.transpose(); cout<<endl;
        cout<<"solve time:    "  << solve_time << " (mu s)" << endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
