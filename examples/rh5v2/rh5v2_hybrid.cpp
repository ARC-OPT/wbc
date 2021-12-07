#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <core/RobotModelConfig.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <Eigen/QR>

using namespace wbc;
using namespace std;

// For testing you have to update the full system state
// Solver output should only contain the actuated joints
// Use hyrodyn forward task?

base::commands::Joints evaluateRobotModel(RobotModelPtr robot_model){

    string root = "RH5_Root_Link";
    string tip  = "LLAnklePitch_Link";
    base::samples::Joints joint_state;
    joint_state.names = std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->hyrodynHandle()->jointnames_independent;
    for(auto n : std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->hyrodynHandle()->jointnames_independent){
        base::JointState js;
        js.position = js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    joint_state["LLKnee"].position = 1.5;
    joint_state["LLAnklePitch"].position = -0.7;

    robot_model->update(joint_state);

    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    VelocitySceneQuadraticCost scene(robot_model, solver);
    std::vector<ConstraintConfig> wbc_config;
    wbc_config.push_back(ConstraintConfig("cart_pos_ctrl", 0,
                                          root,
                                          tip,
                                          root, 1));
    if(!scene.configure(wbc_config))
        abort();
    base::samples::RigidBodyStateSE3 ref;
    ref.twist.linear.setZero();
    ref.twist.angular.setZero();
    ref.twist.linear[2] = -0.1;
    scene.setReference("cart_pos_ctrl", ref);
    HierarchicalQP qp = scene.update();
    return scene.solve(qp);
}

int main(){

    RobotModelHyrodyn robot_model;
    RobotModelConfig config;
    config.file = "../../../models/rh5v2/urdf/rh5v2_hybrid.urdf";
    config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    if(!robot_model.configure(config))
        abort();

    base::samples::Joints joint_state;
    joint_state.names = robot_model.hyrodynHandle()->jointnames_independent;
    for(auto n : robot_model.hyrodynHandle()->jointnames_independent){
        base::JointState js;
        js.position = js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    joint_state["ALShoulder1"].position = -1.0;
    joint_state["ALShoulder2"].position = 1.0;
    joint_state["ALElbow"].position = -1.0;

    joint_state["ARShoulder1"].position = -1.0;
    joint_state["ARShoulder2"].position = 1.0;
    joint_state["ARElbow"].position = -1.0;
    robot_model.update(joint_state);

    base::samples::Joints full_joint_state = robot_model.jointState(robot_model.jointNames());
    for(int i = 0; i < robot_model.hyrodynHandle()->jointnames_spanningtree.size(); i++)
        cout<<robot_model.hyrodynHandle()->jointnames_spanningtree[i]<<": "<<robot_model.hyrodynHandle()->Q[i]<<endl;
    return 0;
}
