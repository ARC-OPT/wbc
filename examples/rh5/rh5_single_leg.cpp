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

    // Configure serial robot model
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    config.joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"};
    config.actuated_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"};
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    if(!robot_model->configure(config))
        return -1;

    vector<string> ind_joint_names = config.joint_names; // Independent joints are identical with the controlled joints in WBC

    // Configure solver
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene
    ConstraintConfig cart_constraint("left_leg_posture", 0, "RH5_Root_Link", "LLAnkle_FT", "RH5_Root_Link", 1);
    VelocitySceneQuadraticCost scene(robot_model, solver);
    if(!scene.configure({cart_constraint}))
        return -1;

    // Configure Cartesian controller
    CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1));

    // Initial joint state
    uint nj = ind_joint_names.size();
    base::samples::Joints joint_state;
    base::VectorXd init_q(nj);
    init_q << 0,0,-0.2,0.4,0,-0.2;
    joint_state.resize(nj);
    joint_state.names = ind_joint_names;
    for(size_t i = 0; i < nj; i++)
        joint_state[i].position = init_q[i];
    joint_state.time = base::Time::now();

    // Reference Pose
    base::samples::RigidBodyStateSE3 setpoint, feedback, ctrl_output;
    setpoint.pose.position = base::Vector3d(0, 0, -0.6);
    setpoint.pose.orientation = base::Quaterniond(0,-1,0,0);
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = 0.001; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-3){

        // Update robot model
        robot_model->update(joint_state);

        // Update controllers
        feedback = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints
        scene.setReference(cart_constraint.name, ctrl_output);

        // Update WBC scene and solve
        HierarchicalQP hqp = scene.update();
        solver_output = scene.solve(hqp);

        // Update joint state
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
