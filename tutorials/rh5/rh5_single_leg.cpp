#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <controllers/CartesianPosPDController.hpp>

using namespace wbc;
using namespace std;
using namespace qpOASES;
using namespace ctrl_lib;

/**
 * Velocity-based example, Cartesian position control on 6 dof leg of the RH5 humanoid (fixed base/fully actuated, serial robot model). In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}_w\dot{\mathbf{q}} - \mathbf{v}_d\|_2\\
 *        \mathbf{\dot{q}} & & \\
 *           s.t.  & \dot{\mathbf{q}}_m \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_M& \\
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
 * \f$\mathbf{K}_p\f$ - Proportional feed forward gain matrix<br>
 * \f$\mathbf{\dot{q}}_m,\mathbf{\dot{q}}_M\f$ - Joint velocity limits<br>
 *
 * The robot ankle is supposed to move to a fixed target pose. The QP is solved using the QPOases solver. Note that the robot has a fixed base here and there are no rigid contacts that have to be considered.
 * In the example, the Hyrodyn robot model is used, however, with a serial URDF model, i.e., the output velocities are in independent joint space.
 */
int main(){

    double dt = 0.001;

    // Create Hyrodyn based robot model.  In this case, we use the Hyrodyn-based model, which allows handling of parallel mechanisms.
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();

    // Configure the model. We pass a serial model description to RobotModelHyrodyn, which means that the solution will be identical
    // to the one obtained by  RobotModelKDL. RobotModelHyrodyn assumes that you specify correctly all joints (actuated and unactuated).
    // Also, we have to pass a submechanism files, which describes the parallel structures inside the robot architecture
    // In this case, there are not parallel structure, since we use the abstract model
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    if(!robot_model->configure(config))
        return -1;

    // Independent joint names. Since the model is serial and we don' t have a floating base:
    // actuated joints == independend joints == all joints
    vector<string> ind_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"};

    // Configure solver, use QPOASES in this case
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene, use VelocitySceneQuadraticCost in this case
    ConstraintConfig cart_constraint;
    cart_constraint.name = "left_leg_posture";     // unique id
    cart_constraint.type = cart;                   // Cartesian or joint space task?
    cart_constraint.priority = 0;                  // Priority, 0 - highest prio
    cart_constraint.root = "RH5_Root_Link";        // Root link of the kinematic chain to consider for this task
    cart_constraint.tip = "LLAnkle_FT";            // Tip link of the kinematic chain to consider for this task
    cart_constraint.ref_frame = "RH5_Root_Link";   // In what frame is the task specified?
    cart_constraint.activation = 1;                // (0..1) initial task activation. 1 - Task should be active initially
    cart_constraint.weights = vector<double>(6,1); // Task weights. Can be used to balance the relativ importance of the task variables (e.g. position vs. orienration)
    VelocitySceneQuadraticCost scene(robot_model, solver, dt);
    if(!scene.configure({cart_constraint}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1));

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    uint nj = ind_joint_names.size();
    base::samples::Joints joint_state;
    base::VectorXd init_q(nj);
    init_q << 0,0,-0.2,0.4,0,-0.2;
    joint_state.resize(nj);
    joint_state.names = ind_joint_names;
    for(size_t i = 0; i < nj; i++)
        joint_state[i].position = init_q[i];
    joint_state.time = base::Time::now();

    // Choose a valid reference pose x_r, which is defined in cart_constraint.ref_frame and defines the desired pose of
    // the cart_constraint.ref_tip frame. The pose will be passed as setpoint to the controller.
    base::samples::RigidBodyStateSE3 setpoint, feedback, ctrl_output;
    setpoint.pose.position = base::Vector3d(0, 0, -0.6);
    setpoint.pose.orientation = base::Quaterniond(0,-1,0,0);
    setpoint.frame_id = cart_constraint.ref_frame;
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    double loop_time = dt; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-3){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        feedback = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(cart_constraint.name, ctrl_output);

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
        cout<<"Joint Names:   "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
