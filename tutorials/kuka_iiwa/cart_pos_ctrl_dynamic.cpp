#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/acceleration_tsid/AccelerationSceneTSID.hpp>
#include <tools/JointIntegrator.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;
using namespace base;

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
    // Control rate
    double dt = 1e-3;

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
    vector<TaskConfig> wbc_config;
    wbc_config.push_back(TaskConfig("cart_pos_ctrl",  0, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", "kuka_lbr_l_link_0", 1.0));
    if(!scene.configure(wbc_config))
        return -1;

    // Choose an initial joint state. Since we use acceleration-based WBC here, we have to pass the velocities as well
    samples::Joints joint_state;
    uint nj = robot_model->noOfJoints();
    joint_state.resize(nj);
    joint_state.names = robot_model->jointNames();
    for(uint i = 0; i < nj; i++)
        joint_state[i].position = joint_state[i].speed = 0.1;
    joint_state.time = Time::now();

    // Configure Cartesian controller. The controller implements the following control law:
    //
    //    a_d = kf*a_r +  kd*(v_r - v) + kp(x_r - x).
    //
    // As we don't use feed forward acceleration here, we can ignore the factor kf.
    CartesianPosPDController ctrl;
    ctrl.setPGain(base::Vector6d::Constant(100));
    ctrl.setDGain(base::Vector6d::Constant(30));
    ctrl.setFFGain(base::Vector6d::Constant(1));

    // Choose a valid reference pose x_r
    samples::RigidBodyStateSE3 setpoint;
    setpoint.pose.orientation.setIdentity();
    setpoint.twist.angular.setZero();

    // Run control loop
    JointIntegrator integrator;
    for(double t = 0; t < 10; t+=dt){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        robot_model->update(joint_state);

        // Generate circular end effector trajectory in xy plane
        setpoint.pose.position = base::Vector3d(0.1*sin(0.3*2*M_PI*t), 0.1*cos(0.3*2*M_PI*t),  1.138);
        setpoint.twist.linear  = base::Vector3d(0.1*cos(0.3*2*M_PI*t), -0.1*sin(0.3*2*M_PI*t), 0.0);

        // Update controllers, left arm: Follow sinusoidal trajectory
        // setpoint_left.pose.position[0] = 0.522827 + 0.1*sin(t);
        // setpoint_left.twist.linear[0] = 0.1*cos(t);
        // setpoint_left.acceleration.linear[0] = -0.1*sin(t);
        samples::RigidBodyStateSE3 feedback = robot_model->rigidBodyState(wbc_config[0].root, wbc_config[0].tip);
        samples::RigidBodyStateSE3 ctrl_output = ctrl.update(setpoint, feedback);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(wbc_config[0].name, ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint acceleration/torque that achieves the task space acceleration demanded by the controllers
        commands::Joints solver_output = scene.solve(hqp);

        // Integrate once to get joint velocity from solver output
        integrator.integrate(joint_state,solver_output,dt);

        // Update joint state by integration again
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_state[i].position = solver_output[i].position;
            joint_state[i].speed = solver_output[i].speed;
        }
        printf("setpoint: x: %2.4f y: %2.4f z: %2.4f\n", setpoint.pose.position(0), setpoint.pose.position(1), setpoint.pose.position(2));
        printf("setpoint: qx: %2.4f qy: %2.4f qz: %2.4f qw: %2.4f\n\n", setpoint.pose.orientation.x(), setpoint.pose.orientation.y(), setpoint.pose.orientation.z(), setpoint.pose.orientation.w());
        printf("feedback: x: %2.4f y: %2.4f z: %2.4f\n", feedback.pose.position(0), feedback.pose.position(1), feedback.pose.position(2));
        printf("feedback: qx: %2.4f qy: %2.4f qz: %2.4f qw: %2.4f\n\n", feedback.pose.orientation.x(), feedback.pose.orientation.y(), feedback.pose.orientation.z(), feedback.pose.orientation.w());

        printf("Solver output: "); printf("\n");
        printf("Joint Names:   "); for(uint i = 0; i < nj; i++) printf("%s ", solver_output.names[i].c_str()); printf("\n");
        printf("Velocity:      "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output[i].speed); printf("\n");
        printf("Acceleration:  "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output[i].acceleration); printf("\n");
        printf("Torque:        "); for(uint i = 0; i < nj; i++) printf("%2.4f ", solver_output[i].effort); printf("\n");
        printf("---------------------------------------------------------------------------------------------\n\n");
        usleep(dt * 1e6);
    }

    return 0;
}
