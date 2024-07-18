#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/acceleration_tsid/AccelerationSceneTSID.hpp>
#include <tools/JointIntegrator.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/CartesianAccelerationTask.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;
using namespace wbc;

/**
 * Acceleration-based example, Cartesian position control of the KUKA iiwa robot (fixed base, no contacts).
 * In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}\ddot{\mathbf{q}} - \dot{\mathbf{v}}_d + \dot{\mathbf{J}}\dot{\mathbf{q}}\|_2\\
 *        \mathbf{\ddot{q}},\mathbf{\tau} & & \\
 *           s.t.  & \mathbf{H}\mathbf{\ddot{q}} - \mathbf{\tau} - \mathbf{J}_{c,i}^T\mathbf{f} = -\mathbf{h}, \, \forall i & \\
 *                 & \mathbf{\tau}_m \leq \mathbf{\tau} \leq \mathbf{\tau}_M& \\
 *        \end{array}
 *  \f]
 * where
 *  \f[
 *        \dot{\mathbf{v}}_d = \dot{\mathbf{v}}_r + \mathbf{K}_d(\mathbf{v}_r-\mathbf{v}) + \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
 *  \f]
 * \f$\ddot{\mathbf{q}}\f$ - Vector of robot joint accelerations<br>
 * \f$\dot{\mathbf{v}}_{d}\f$ - desired spatial acceleration / controller output <br>
 * \f$\mathbf{\tau}\f$ - actuation forces/torques<br>
 * \f$\mathbf{J}\f$ - task Jacobian <br>
 * \f$\mathbf{H}\f$ - Joint space inertia matrix<br>
 * \f$\mathbf{h}\f$ - bias forces/torques<br>
 * \f$\mathbf{\tau}_m,\mathbf{\tau}_M\f$ - Joint force/torque limits<br>
 * \f$\mathbf{x}_r, \mathbf{x}\f$ - Reference pose, actual pose<br>
 * \f$\mathbf{v}_r, \mathbf{v}\f$ - Reference spatial velocity, actual spatial velocity<br>
 * \f$\dot{\mathbf{v}}_r\f$ - Reference spatial acceleration (feed forward)<br>
 * \f$\mathbf{K}_p, \mathbf{K}_d\f$ - Proportional, derivative gain matrix<br>
 *
 * The end effector is supposed to follow a sinusoidal trajectory. The solver computes the joint accelerations and forces/torques required
 * to generate the desired spatial accelerations given by the Cartesian controller.
 */
int main()
{
    double dt = 0.01;

    // Create robot model, use hyrodyn-based model in this case
    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();

    // Configure robot model. We configure a serial model without parallel mechanisms.
    // Blacklist the finger joints, as they are not relevant and may slow down the solver
    RobotModelConfig config("../../../models/kuka/urdf/kuka_iiwa.urdf");
    if(!robot_model->configure(config))
        return -1;

    // Configure solver, use QPOases
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure the AccelerationSceneTSID scene. This scene computes joint accelerations, joint torques and contact wrenches as output.
    // Pass two tasks here: Left arm Cartesian pose and right arm Cartesian pose.
    AccelerationSceneTSID scene(robot_model, solver, dt);
    CartesianAccelerationTaskPtr cart_task;
    cart_task = make_shared<CartesianAccelerationTask>(TaskConfig("cart_pos_ctrl",0,{1,1,1,1,1,1},1),
                                                       "kuka_lbr_l_tcp",
                                                       "kuka_lbr_l_link_0",
                                                       robot_model->nj());
    if(!scene.configure({cart_task}))
        return -1;

    // Choose an initial joint state. Since we use acceleration-based WBC here, we have to pass the velocities as well
    types::JointState joint_state;
    uint nj = robot_model->na();
    joint_state.resize(nj);
    joint_state.position.setConstant(0.1);
    joint_state.velocity.setConstant(0.1);

    // Configure Cartesian controller. The controller implements the following control law:
    //
    //    a_d = kf*a_r +  kd*(v_r - v) + kp(x_r - x).
    //
    // As we don't use feed forward acceleration here, we can ignore the factor kf.
    CartesianPosPDController ctrl;
    Eigen::VectorXd p_gain(6),d_gain(6),ff_gain(6);
    p_gain.setConstant(10); // Stiffness
    d_gain.setConstant(30); // Damping
    ff_gain.setConstant(1); // Feed forward
    ctrl.setPGain(p_gain);
    ctrl.setDGain(d_gain);
    ctrl.setDGain(ff_gain);

    // Choose a valid reference pose x_r
    types::Pose ref_pose, pose;
    types::Twist ref_twist, twist;
    types::SpatialAcceleration ctrl_output, ref_acc;
    ref_pose.position = Eigen::Vector3d(0.0, 0.0, 1.0);
    ref_pose.orientation.setIdentity();
    ref_twist.setZero();
    ref_acc.setZero();

    // Run control loop
    JointIntegrator integrator;
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    for(double t = 0; t < 10; t+=loop_time){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration);

        // Update controllers, left arm: Follow sinusoidal trajectory
        // setpoint_left.pose.position[0] = 0.522827 + 0.1*sin(t);
        // setpoint_left.twist.linear[0] = 0.1*cos(t);
        // setpoint_left.acceleration.linear[0] = -0.1*sin(t);
        pose = robot_model->pose(cart_task->tipFrame());
        twist = robot_model->twist(cart_task->tipFrame());
        ctrl_output = ctrl.update(ref_pose,ref_twist,ref_acc,pose,twist);
        ref_pose.position[2] = 1.0 + 0.1*sin(t);
        ref_twist.linear[2] = 0.1*cos(t);
        ref_acc.linear[2] = -0.1*sin(t);

        // Update tasks. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint acceleration/torque that achieves the task space acceleration demanded by the controllers
        solver_output = scene.solve(hqp);

        // Integrate once to get joint velocity from solver output
        integrator.integrate(joint_state,solver_output,loop_time,types::CommandMode::ACCELERATION);

        // Update joint state
        joint_state.position = solver_output.position;
        joint_state.velocity = solver_output.velocity;

        printf("setpoint: x: %2.4f y: %2.4f z: %2.4f\n", ref_pose.position(0), ref_pose.position(1), ref_pose.position(2));
        printf("setpoint: qx: %2.4f qy: %2.4f qz: %2.4f qw: %2.4f\n\n", ref_pose.orientation.x(), ref_pose.orientation.y(), ref_pose.orientation.z(), ref_pose.orientation.w());
        printf("feedback: x: %2.4f y: %2.4f z: %2.4f\n", pose.position(0), pose.position(1), pose.position(2));
        printf("feedback: qx: %2.4f qy: %2.4f qz: %2.4f qw: %2.4f\n\n", pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());

        printf("Solver output: "); printf("\n");
        printf("Joint Names:   "); for(uint i = 0; i < nj; i++) printf("%s ", robot_model->jointNames()[i].c_str()); printf("\n");
        printf("Velocity:      "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output.velocity[i]); printf("\n");
        printf("Acceleration:  "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output.acceleration[i]); printf("\n");
        printf("Torque:        "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output.effort[i]); printf("\n");
        printf("---------------------------------------------------------------------------------------------\n\n");
        usleep(loop_time * 1e6);
    }

    return 0;
}
