#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace wbc;

/**
 * Simple Velocity-based example, Cartesian position control on a kuka iiwa 7 dof arm. In contrast to the cart_pos_ctrl_hls example,
 * a QP solver (qpoases) is used to compute the solution. In the example the following problem is solved:
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
 * The robot end effector is supposed to move to a fixed target pose. The QP is solved using the QPOases solver. In contrast to the cart_pos_ctrl_hls example,
 * the solver output will always be within the joint velocity limits, which are defined in the kuka iiwa URDF file.
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
    TaskConfig cart_task;
    cart_task.name       = "cart_pos_ctrl";      // Unique identifier
    cart_task.type       = cart;                 // Cartesian or joint space task?
    cart_task.priority   = 0;                    // Priority, 0 - highest prio
    cart_task.root       = "kuka_lbr_l_link_0";  // Root link of the kinematic chain to consider for this task
    cart_task.tip        = "kuka_lbr_l_tcp";     // Tip link of the kinematic chain to consider for this task
    cart_task.ref_frame  = "kuka_lbr_l_link_0";  // In what frame is the task specified?
    cart_task.activation = 1;                    // (0..1) initial task activation. 1 - Task should be active initially
    cart_task.weights    = vector<double>(6,1);  // Task weights. Can be used to balance the relativ importance of the task variables (e.g. position vs. orienration)
    VelocitySceneQP scene(robot_model, solver, 1e-3);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    ctrl_lib::CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1));

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    base::samples::Joints joint_state;
    uint nj = robot_model->noOfJoints();
    joint_state.resize(nj);
    joint_state.names = robot_model->jointNames();
    for(uint i = 0; i < nj; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now();

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    base::samples::RigidBodyStateSE3 setpoint, ctrl_output, feedback;
    setpoint.pose.position = base::Vector3d(0.0, 0.0, 0.95);
    setpoint.pose.orientation.setIdentity();
    setpoint.frame_id = cart_task.ref_frame;
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = dt; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        auto s = std::chrono::high_resolution_clock::now();
        robot_model->update(joint_state);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        feedback = robot_model->rigidBodyState(cart_task.root, cart_task.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(cart_task.name, ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();        

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);
        auto e = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(e-s).count();

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
        cout<<"solve time:    " << solve_time << " (mu s)" << endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
