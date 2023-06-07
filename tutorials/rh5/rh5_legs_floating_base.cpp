#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/rbdl/RobotModelRBDL.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tools/JointIntegrator.hpp>
#include <unistd.h>

using namespace wbc;
using namespace std;
using namespace qpOASES;
using namespace ctrl_lib;

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
    RobotModelPtr robot_model = std::make_shared<RobotModelRBDL>();

    // Configure a serial robot model with floating base and two contact points: {"LLAnkle_FT", "LRAnkle_FT"}.
    // Note that the joint names have to contain {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
    // "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"} in addition to the actuated joints.
    // Also a valid initial state (pose/twist/acceleration) of the floating base should be passed
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    config.floating_base = true;
    config.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config.contact_points.elements = {wbc::ActiveContact(1,0.6),wbc::ActiveContact(1,0.6)};
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
    TaskConfig cart_task;
    cart_task.name = "com_position";
    cart_task.type = cart;
    cart_task.priority = 0;
    cart_task.root = "world";
    cart_task.tip = "RH5_Root_Link";
    cart_task.ref_frame = "world";
    cart_task.activation = 1;
    cart_task.weights = vector<double>(6,1);
    VelocitySceneQP scene(robot_model, solver, dt);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(3));

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed.
    // Note that you don't have to pass the floating base pose here.
    uint nj = robot_model->noOfActuatedJoints();
    base::VectorXd q(nj);
    q << 0,0,-0.35,0.64,0,-0.27, 0,0,-0.35,0.64,0,-0.27;
    base::samples::Joints joint_state;
    joint_state.names = robot_model->actuatedJointNames();
    joint_state.resize(nj);
    for(size_t i = 0; i < nj; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = joint_state[i].acceleration = 0;
    }
    joint_state.time = base::Time::now();

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    base::samples::RigidBodyStateSE3 setpoint, feedback, ctrl_output;
    setpoint.pose.position = base::Vector3d(-0.0,0,0.7);
    setpoint.pose.orientation = base::Quaterniond(1,0,0,0);
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = dt; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        // Note that you have to pass the floating base state as well now!
        robot_model->update(joint_state, floating_base_state);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        feedback = robot_model->rigidBodyState(cart_task.root, cart_task.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(cart_task.name, ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Update joint state by simple integration using the solver output
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_state[i].position += solver_output[i].speed * loop_time;
            joint_state[i].speed = solver_output[i].speed;
        }

        // Update floating base pose, use an over-simplistic estimation
        base::Pose t1 = robot_model->rigidBodyState("world", "RH5_Root_Link").pose;
        base::Pose t2 = robot_model->rigidBodyState("world", "LLAnkle_FT").pose;
        base::Pose t3;
        t3.fromTransform(t2.toTransform().inverse()*t1.toTransform());
        floating_base_state.pose.position = -t3.position;
        floating_base_state.pose.position[0]=floating_base_state.pose.position[1]=0;

        cout<<"setpoint: x: "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx: "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x: "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx: "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(uint i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
