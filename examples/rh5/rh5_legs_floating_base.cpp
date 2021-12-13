#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/kdl/RobotModelKDL.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
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

    // Configure serial robot model with floating base and two contact points: {"LLAnkle_FT", "LRAnkle_FT"
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    config.joint_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                          "floating_base_rot_x",   "floating_base_rot_y",   "floating_base_rot_z",
                          "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch",
                          "LRHip1", "LRHip2", "LRHip3", "LRKnee", "LRAnkleRoll", "LRAnklePitch"};
    config.actuated_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch",
                                   "LRHip1", "LRHip2", "LRHip3", "LRKnee", "LRAnkleRoll", "LRAnklePitch"};
    config.floating_base = true;
    config.world_frame_id = "world";
    config.floating_base_state = floating_base_state;
    config.contact_points = {"LLAnkle_FT", "LRAnkle_FT"};
    RobotModelPtr robot_model = std::make_shared<RobotModelKDL>();
    if(!robot_model->configure(config))
        return -1;

    // Configure solver
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    Options options = std::dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene
    ConstraintConfig cart_constraint("com_position", 0, "world", "RH5_Root_Link", "world", 1);
    VelocitySceneQuadraticCost scene(robot_model, solver);
    if(!scene.configure({cart_constraint}))
        return -1;

    // Initial joint state
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

    // Configure Cartesian controller
    CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(3));

    // Reference Pose
    base::samples::RigidBodyStateSE3 setpoint, feedback, ctrl_output;
    setpoint.pose.position = base::Vector3d(-0.0,0,0.6);
    setpoint.pose.orientation = base::Quaterniond(1,0,0,0);
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = 0.001; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){
        // Update robot model
        robot_model->update(joint_state, floating_base_state);

        // Update controllers
        feedback = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints
        scene.setReference(cart_constraint.name, ctrl_output);

        // Update WBC scene and solve
        HierarchicalQP hqp = scene.update();
        solver_output = scene.solve(hqp);

        // Update joint state
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_state[i].position += solver_output[i].speed * loop_time;
            joint_state[i].speed = solver_output[i].speed;
        }

        // Update floating base pose, use a simplistic estimation
        floating_base_state.pose.position = robot_model->rigidBodyState("LLAnkle_FT", "RH5_Root_Link").pose.position;
        floating_base_state.pose.position*=-1;
        floating_base_state.pose.position[0]=floating_base_state.pose.position[1]=0;

        cout<<"setpoint: x: "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx: "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x: "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx: "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
