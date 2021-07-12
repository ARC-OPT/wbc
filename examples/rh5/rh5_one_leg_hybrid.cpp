#include <robot_models/RobotModelHyrodyn.hpp>
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
    joint_state.names = std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->jointnames_independent;
    for(auto n : std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->jointnames_independent){
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
                                          "RH5_Root_Link",
                                          "LLAnklePitch_Link",
                                          "RH5_Root_Link", 1));
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

    RobotModelPtr robot_model_hybrid = std::make_shared<RobotModelHyrodyn>();
    RobotModelConfig config_hybrid("../../../../../bundles/wbc_examples/config/models/rh5/urdf/rh5_single_leg_hybrid.urdf",
                                   {"LLHip1", "LLHip2",
                                    "LLHip3", "LLHip3_B11", "LLHip3_Act1",
                                    "LLKnee", "LLKnee_B11", "LLKnee_Act1",
                                    "LLAnkleRoll", "LLAnklePitch", "LLAnkle_E11", "LLAnkle_E21", "LLAnkle_B11", "LLAnkle_B12", "LLAnkle_Act1", "LLAnkle_B21", "LLAnkle_B22", "LLAnkle_Act2"},
                                   {"LLHip1", "LLHip2", "LLHip3_Act1","LLKnee_Act1", "LLAnkle_Act1", "LLAnkle_Act2"});
    config_hybrid.submechanism_file = "../../../models/hyrodyn/rh5/rh5_one_leg_hybrid.yml";
    if(!robot_model_hybrid->configure(config_hybrid))
        abort();

    RobotModelPtr robot_model_serial = std::make_shared<RobotModelHyrodyn>();
    RobotModelConfig config_serial("../../../models/urdf/rh5/rh5_one_leg.urdf",
                                   {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                   {"LLHip1", "LLHip2", "LLHip3","LLKnee", "LLAnkleRoll", "LLAnklePitch"});
    config_serial.submechanism_file = "../../../models/hyrodyn/rh5/rh5_one_leg.yml";
    if(!robot_model_serial->configure(config_serial))
        abort();

    cout<<"******************** HYBRID MODEL *****************"<<endl;
    base::commands::Joints solver_output = evaluateRobotModel(robot_model_hybrid);
    cout<< "Solver output" << endl;
    for(auto n : solver_output.names)
        cout << n << ": " << solver_output[n].speed << endl;
    for(int i = 0; i < solver_output.size(); i++)
        std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->ud[i] = solver_output[i].speed;
    std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->calculate_forward_system_state();

    cout<< "Solver output projected to independent joint space" << endl;
    std::cout<<std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->yd.transpose()<<endl;

    cout<<"******************** SERIAL MODEL *****************"<<endl;
    solver_output = evaluateRobotModel(robot_model_serial);
    cout<< "Solver output" << endl;
    for(auto n : solver_output.names)
        cout << n << ": " << solver_output[n].speed << endl;
    return 0;
}
