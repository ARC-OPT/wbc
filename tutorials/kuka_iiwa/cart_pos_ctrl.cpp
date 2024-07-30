#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/CartesianVelocityTask.hpp>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace wbc;

/**
 * Simple Velocity-based example, Cartesian position control on a kuka iiwa 7 dof arm. In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize & \| \mathbf{J_w\dot{q}}-\mathbf{v}_{d} \|_2& \\
 *            \mathbf{\dot{q}} & & \\
 *           s.t. & \mathbf{\dot{q}}_m \leq \mathbf{\dot{q}} \leq  \mathbf{\dot{q}}_M
 *        \end{array}
 *  \f]
 * where
 *  \f[
 *        \mathbf{v}_d = \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
 *  \f]
 * \f$\dot{\mathbf{q}}\f$ - Vector of robot joint velocities<br>
 * \f$\mathbf{v}_{d}\f$ - Desired spatial velocity / controller output <br>
 * \f$\mathbf{J}_w=\mathbf{W}\mathbf{J}\f$ - Weighted task Jacobian <br>
 * \f$\mathbf{W}\f$ - Diagonal weight matrix<br>
 * \f$\mathbf{x}_r, \mathbf{x}\f$ - Reference pose, actual pose<br>
 * \f$\mathbf{K}_p\f$ - Proportional gain matrix<br>
 * \f$\mathbf{\dot{q}}_m,\mathbf{\dot{q}}_M\f$ - Joint velocity limits<br>
 *
 * The robot end effector is supposed to move to a fixed target pose. The QP is solved using the QPOases solver. The solver output will always be within the joint velocity limits, which are defined in the kuka iiwa URDF file.
 */
int main(){

    double dt = 0.01;

    // Create a robot model. Each robot model is derived from a common RobotModel class, as it will be passed to the WBC scene later on.
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

    // Create a solver. In this case, we use the QPOases solver, which is a numerical QP solver. It allows only a single priority
    // level. However, prioritization can be achieved through the task weights. Also, it allows hard joint constraints,
    // like e.g., joint velocity limits.
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    qpOASES::Options options;
    options.setToDefault();
    options.printLevel = qpOASES::PL_NONE;
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(100);

    // Configure WBC Scene. Use the Scene VelocitySceneQuadraticCost here. This scene implements tasks as part of the cost function
    // of the QP and defines the maximum joint velocities (taken from the URDF) as hard inequality constraints. The solution will
    // behave similarly in task space. However, as you wil see, the joint velocities will not exceed the joint limits defined in the URDF file.

    CartesianVelocityTaskPtr cart_task;
    cart_task = make_shared<CartesianVelocityTask>(TaskConfig("cart_pos_ctrl",0,vector<double>(6,1),1),
                                                   "kuka_lbr_l_tcp",
                                                   "kuka_lbr_l_link_0",
                                                   robot_model->nj());
    VelocitySceneQP scene(robot_model, solver, 1e-3);
    if(!scene.configure({cart_task}))
        return -1;

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

    // Run control loop
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    while((ref_pose.position - pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        auto s = std::chrono::high_resolution_clock::now();
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
        auto e = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(e-s).count();

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
        cout<<"solve time:    " << solve_time << " (us)" << endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
