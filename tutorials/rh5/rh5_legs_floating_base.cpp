#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/SpatialVelocityTask.hpp>
#include <tools/JointIntegrator.hpp>
#include <unistd.h>
#include <chrono>

using namespace wbc;
using namespace std;
using namespace qpOASES;
using namespace wbc;

/**
 * Velocity-based example, Cartesian position control on two 6 dof legs of the RH5 humanoid including floating base and two contact points (feet).
 * In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}_w\dot{\mathbf{q}} - \mathbf{v}_d \|_2\\
 *        \mathbf{\dot{q}} & & \\
 *                 & \mathbf{J}_{c,i}\dot{\mathbf{q}} = 0, \, \forall i& \\
 *                 & \dot{\mathbf{q}}_m \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_M& \\
 *        \end{array}
 *  \f]
 * where
 *  \f[
 *        \mathbf{v}_d = \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
 *  \f]
 * \f$\ddot{\mathbf{q}}\f$ - Vector of robot joint accelerations<br>
 * \f$\dot{\mathbf{v}}_{d}\f$ - Desired spatial acceleration / controller output <br>
 * \f$\mathbf{J}_w\f$ - Weighted task Jacobian <br>
 * \f$\mathbf{W}\f$ - Diagonal weight matrix<br>
 * \f$\mathbf{J}_{c,i}\f$ - Contact Jacobian of i-th contact point<br>
 * \f$\mathbf{x}_r, \mathbf{x}\f$ - Reference pose, actual pose<br>
 * \f$\mathbf{K}_p\f$ - Derivative gain matrix<br>
 * \f$\mathbf{\dot{q}}_m,\mathbf{\dot{q}}_M\f$ - Joint velocity limits<br>
 *
 * The robot CoM is supposed to move to a fixed target pose. The QP is solved using the QPOases solver. Note that the robot has a floating base and there are
 * two rigid contacts with the environment given by the feet contacting the ground floor. The solver computes the joint velocities
 * required to comply with the desired spatial velocities given by the Cartesian controller.
 */
int main(){

    double dt = 0.001;

    // Create robot model
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();

    // Configure a serial robot model with floating base and two contact points: {"LLAnkle_FT", "LRAnkle_FT"}.
    // Note that the joint names have to contain {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
    // "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"} in addition to the actuated joints.
    // Also a valid initial state (pose/twist/acceleration) of the floating base should be passed
    types::RigidBodyState floating_base_state;
    floating_base_state.pose.position = Eigen::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = Eigen::Quaterniond(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config;
    config.file_or_string = "../../../models/rh5/urdf/rh5_legs.urdf";
    config.floating_base = true;
    config.contact_points = {types::Contact("LLAnkle_FT",1,0.6),types::Contact("LRAnkle_FT",1,0.6)};
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

    SpatialVelocityTaskPtr cart_task;
    cart_task = make_shared<SpatialVelocityTask>(TaskConfig("com_position",0,vector<double>(6,1),1),
                                                   robot_model,
                                                   "RH5_Root_Link",
                                                   "world");
    VelocitySceneQP scene(robot_model, solver, dt);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    Eigen::VectorXd p_gain(6);
    p_gain.segment(0,3).setConstant(3.0);
    p_gain.segment(3,3).setZero();
    controller.setPGain(p_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed.
    // Note that you don't have to pass the floating base pose here.
    uint nj = robot_model->na();
    Eigen::VectorXd q(nj);
    types::JointState joint_state;
    joint_state.resize(nj);
    joint_state.position << 0,0,-0.35,0.64,0,-0.27, 0,0,-0.35,0.64,0,-0.27;
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::Pose ref_pose, pose;
    types::Twist ref_twist, ctrl_output;
    ref_pose.position = Eigen::Vector3d(-0.0,0,0.7);
    ref_pose.orientation = Eigen::Quaterniond(1,0,0,0);
    ref_twist.setZero();
    pose.position.setZero();
    pose.orientation.setIdentity();

    // Run control loop
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
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

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        robot_model->pose("LLAnkle_FT");
        pose = robot_model->pose(cart_task->tipFrame());
        ctrl_output = controller.update(ref_pose, ref_twist, pose);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Update joint state by simple integration using the solver output
        joint_state.position += solver_output.velocity * loop_time;
        joint_state.velocity = solver_output.velocity;

        auto e = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(e-s).count();

        // Update floating base pose, use an over-simplistic estimation
        types::Pose pose_1 = robot_model->pose("RH5_Root_Link");
        types::Pose pose_2 = robot_model->pose("LLAnkle_FT");
        types::Pose pose_3;
        Eigen::Affine3d t1,t2,t3;
        t1 = pose_1.orientation;
        t1.pretranslate(pose_1.position);
        t2 = pose_2.orientation;
        t2.pretranslate(pose_2.position);
        t3 = t2.inverse()*t1;
        floating_base_state.pose.position = -t3.translation();
        floating_base_state.pose.position[0]=floating_base_state.pose.position[1]=0;

        cout<<"setpoint: x: "<<ref_pose.position(0)<<" y: "<<ref_pose.position(1)<<" z: "<<ref_pose.position(2)<<endl;
        cout<<"setpoint: qx: "<<ref_pose.orientation.x()<<" qy: "<<ref_pose.orientation.y()<<" qz: "<<ref_pose.orientation.z()<<" qw: "<<ref_pose.orientation.w()<<endl<<endl;
        cout<<"feedback x: "<<pose.position(0)<<" y: "<<pose.position(1)<<" z: "<<pose.position(2)<<endl;
        cout<<"feedback qx: "<<pose.orientation.x()<<" qy: "<<pose.orientation.y()<<" qz: "<<pose.orientation.z()<<" qw: "<<pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<robot_model->jointNames()[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; cout<<solver_output.velocity.transpose()<<" "; cout<<endl;
        cout<<"solve time:    " << solve_time << " (mu s)" << endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
