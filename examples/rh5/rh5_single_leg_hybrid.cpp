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
 * Velocity-based example, exact same problem as in the rh5_single_leg example. The only difference is that here we use the hybrid robot model, i.e. we
 * consider the parallel structures in the mechanical structure and solve the WBC problem in actuation space. The output is thus not a velocity vector
 * of the independent joints, but the velocities of the actual actuators. In case of the RH5 leg, there are three such parallel structures:
 *
 * - The Hip3 joint is driven by a linear actuator
 * - The Knee joint is driven by a linear actuator
 * - The ankle structure is a parallel mechanism with two linear actuators
 */
int main(){

    // Configure hybrid robot model
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf";
    config.joint_names = {"LLHip1", "LLHip2","LLHip3", "LLHip3_B11", "LLHip3_Act1","LLKnee", "LLKnee_B11", "LLKnee_Act1", "LLAnkleRoll", "LLAnklePitch", "LLAnkle_E11", "LLAnkle_E21", "LLAnkle_B11", "LLAnkle_B12", "LLAnkle_Act1", "LLAnkle_B21", "LLAnkle_B22", "LLAnkle_Act2"};
    config.actuated_joint_names = {"LLHip1", "LLHip2", "LLHip3_Act1","LLKnee_Act1", "LLAnkle_Act1", "LLAnkle_Act2"};
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(config))
        return -1;

    vector<string> ind_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"}; // Independent joints differ from controlled joints in WBC

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
    setpoint.pose.position = base::Vector3d(0,0, -0.6);
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

        // Use Hyrodyn to compute the joint velocity in independent joint space from the solver output, which contains the actuated joints
        hyrodyn::RobotModel_HyRoDyn* rm_hyrodyn = std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->hyrodynHandle();
        for(int i = 0; i < solver_output.size(); i++)
            rm_hyrodyn->ud[i] = solver_output[i].speed;
        rm_hyrodyn->calculate_forward_system_state();

        // Update joint state
        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += rm_hyrodyn->yd[i]*loop_time;

        cout<<"setpoint: x:    "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx:   "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:     "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx:    "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output:  "; cout<<endl;
        cout<<"Joint Names:    "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:       "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
