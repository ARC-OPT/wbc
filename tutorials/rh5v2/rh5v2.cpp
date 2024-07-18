#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/acceleration_tsid/AccelerationSceneTSID.hpp>
#include <tools/JointIntegrator.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/CartesianAccelerationTask.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;

/**
 * Acceleration-based example, Cartesian position control of the RH5v2 arms (fixed base, no contacts).
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
 * Two tasks are implemented: The left end effector is supposed to follow a sinusoidal trajectory, while the right end effector is supposed to
 * maintain a fixed pose. Note that the tasks share the torso joints (BodyPitch/Roll/Yaw). The QP is solved using the QPOases solver.
 * The solver computes the joint accelerations and forces/torques required to generate the desired spatial accelerations given by the Cartesian controllers.
 */
int main()
{
    double dt = 0.01;

    // Create robot model, use hyrodyn-based model in this case
    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();

    // Configure robot model. We configure a serial model without parallel mechanisms.
    // Blacklist the finger joints, as they are not relevant and may slow down the solver
    RobotModelConfig config("../../../models/rh5v2/urdf/rh5v2.urdf");
    config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
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
    CartesianAccelerationTaskPtr cart_task_left, cart_task_right;
    cart_task_left = make_shared<CartesianAccelerationTask>(TaskConfig("cart_ctrl_left",0,{1,1,1,1,1,1},1),
                                                            "ALWristFT_Link",
                                                            "RH5v2_Root_Link",
                                                            robot_model->nj());
    cart_task_right = make_shared<CartesianAccelerationTask>(TaskConfig("cart_ctrl_right",0,{1,1,1,1,1,1},1),
                                                            "ARWristFT_Link",
                                                            "RH5v2_Root_Link",
                                                            robot_model->nj());
    if(!scene.configure({cart_task_left, cart_task_right}))
        return -1;

    // Choose an initial joint state. Since we use acceleration-based WBC here, we have to pass the velocities and
    // accelerations as well
    types::JointState joint_state;
    joint_state.resize(robot_model->na());
    joint_state.position.setZero();
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();

    joint_state.position[robot_model->jointIndex("ALShoulder1")] = joint_state.position[robot_model->jointIndex("ARShoulder1")] = -1.0;
    joint_state.position[robot_model->jointIndex("ALShoulder2")] = joint_state.position[robot_model->jointIndex("ARShoulder2")] = 1.0;
    joint_state.position[robot_model->jointIndex("ALElbow")] = joint_state.position[robot_model->jointIndex("ARElbow")] = -1.0;
    robot_model->update(joint_state.position,
                        joint_state.velocity,
                        joint_state.acceleration);

    // Configure Cartesian controllers. The controllers implement the following control law:
    //
    //    a_d = kf*a_r +  kd*(v_r - v) + kp(x_r - x).
    //
    // As we don't use feed forward acceleration here, we can ignore the factor kf.
    CartesianPosPDController ctrl_left, ctrl_right;
    Eigen::VectorXd p_gain(6),d_gain(6),ff_gain(6);
    p_gain.setConstant(10); // Stiffness
    d_gain.setConstant(30); // Damping
    ff_gain.setConstant(1); // Feed forward
    ctrl_left.setPGain(p_gain);
    ctrl_left.setDGain(d_gain);
    ctrl_left.setDGain(ff_gain);
    ctrl_right.setPGain(p_gain);

    // Target Pose left/right
    types::RigidBodyState setpoint_left, setpoint_right, feedback_left, feedback_right;
    types::SpatialAcceleration ctrl_output_left, ctrl_output_right;
    setpoint_left.pose.position = Eigen::Vector3d(0.522827,0.453543,0.183343);
    setpoint_left.pose.orientation = Eigen::Quaterniond(0.371912,-0.485673,0.725747,-0.314793);
    setpoint_left.twist.setZero();
    setpoint_left.acceleration.setZero();
    setpoint_right.pose.position = Eigen::Vector3d(0.522827, -0.453543, 0.183345);
    setpoint_right.pose.orientation = Eigen::Quaterniond(0.371914, 0.485672, 0.72575, 0.314787);
    setpoint_right.twist.setZero();
    setpoint_right.acceleration.setZero();

    // Run control loop
    JointIntegrator integrator;
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    for(double t = 0; t < 5; t+=loop_time){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration);

        // Update controllers, left arm: Follow sinusoidal trajectory
        setpoint_left.pose.position[0] = 0.522827 + 0.1*sin(t);
        setpoint_left.twist.linear[0] = 0.1*cos(t);
        setpoint_left.acceleration.linear[0] = -0.1*sin(t);
        feedback_left.pose = robot_model->pose(cart_task_left->tipFrame());
        feedback_right.pose = robot_model->pose(cart_task_right->tipFrame());
        feedback_left.twist = robot_model->twist(cart_task_left->tipFrame());
        feedback_right.twist = robot_model->twist(cart_task_right->tipFrame());
        ctrl_output_left = ctrl_left.update(setpoint_left.pose,
                                            setpoint_left.twist,
                                            setpoint_left.acceleration,
                                            feedback_left.pose,
                                            feedback_left.twist);
        ctrl_output_right = ctrl_right.update(setpoint_right.pose,
                                              setpoint_right.twist,
                                              setpoint_right.acceleration,
                                              feedback_right.pose,
                                              feedback_right.twist);

        // Update tasks. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task_left->setReference(ctrl_output_left);
        cart_task_right->setReference(ctrl_output_right);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint acceleration/torque that achieves the task space acceleration demanded by the controllers
        solver_output = scene.solve(hqp);

        // Integrate once to get joint velocity from solver output
        integrator.integrate(joint_state,solver_output,loop_time, types::CommandMode::ACCELERATION);

        // Update joint state by integration again
        joint_state.position += solver_output.velocity * loop_time;
        joint_state.velocity = solver_output.velocity;

        cout<<"setpoint left: x: "<<setpoint_left.pose.position(0)<<" y: "<<setpoint_left.pose.position(1)<<" z: "<<setpoint_left.pose.position(2)<<endl;
        cout<<"setpoint left: qx: "<<setpoint_left.pose.orientation.x()<<" qy: "<<setpoint_left.pose.orientation.y()<<" qz: "<<setpoint_left.pose.orientation.z()<<" qw: "<<setpoint_left.pose.orientation.w()<<endl<<endl;
        cout<<"feedback left x: "<<feedback_left.pose.position(0)<<" y: "<<feedback_left.pose.position(1)<<" z: "<<feedback_left.pose.position(2)<<endl;
        cout<<"feedback left qx: "<<feedback_left.pose.orientation.x()<<" qy: "<<feedback_left.pose.orientation.y()<<" qz: "<<feedback_left.pose.orientation.z()<<" qw: "<<feedback_left.pose.orientation.w()<<endl<<endl;

        cout<<"setpoint right: x: "<<setpoint_right.pose.position(0)<<" y: "<<setpoint_right.pose.position(1)<<" z: "<<setpoint_right.pose.position(2)<<endl;
        cout<<"setpoint right: qx: "<<setpoint_right.pose.orientation.x()<<" qy: "<<setpoint_right.pose.orientation.y()<<" qz: "<<setpoint_right.pose.orientation.z()<<" qw: "<<setpoint_right.pose.orientation.w()<<endl<<endl;
        cout<<"feedback right x: "<<feedback_right.pose.position(0)<<" y: "<<feedback_right.pose.position(1)<<" z: "<<feedback_right.pose.position(2)<<endl;
        cout<<"feedback right qx: "<<feedback_right.pose.orientation.x()<<" qy: "<<feedback_right.pose.orientation.y()<<" qz: "<<feedback_right.pose.orientation.z()<<" qw: "<<feedback_right.pose.orientation.w()<<endl<<endl;

        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < robot_model->na(); i++) cout<<robot_model->jointNames()[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; cout<<solver_output.velocity.transpose()<<" "; cout<<endl;
        cout<<"Acceleration:  "; cout<<solver_output.acceleration.transpose()<<" "; cout<<endl;
        cout<<"Torque:        "; cout<<solver_output.effort.transpose()<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;
        usleep(loop_time * 1e6);
    }

    return 0;
}
