#include <scenes/VelocityScene.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <boost/filesystem.hpp>
#include "../benchmarks_common.hpp"
#include "../robot_models_common.hpp"

using namespace wbc;
using namespace std;

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean() << " ms +/- " << stdDev(results["scene_update"]) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean() << " ms +/- " << stdDev(results["scene_solve"]) << endl;
}

map<string,base::VectorXd> evaluateVelocitySceneQuadraticCost(RobotModelPtr robot_model, const std::string &root, const std::string &tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure VelocitySceneQuadraticCost");
    return evaluateWBCSceneRandom(scene, n_samples);
}

map<string,base::VectorXd> evaluateAccelerationSceneTSID(RobotModelPtr robot_model, const std::string &root, const std::string &tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");
    return evaluateWBCSceneRandom(scene, n_samples);
}

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelKUKAIiwa("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelKUKAIiwa("hyrodyn");

    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";
    map<string,base::VectorXd> results_kdl_vel = evaluateVelocitySceneQuadraticCost(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl_acc = evaluateAccelerationSceneTSID(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn, root, tip, n_samples);

    toCSV(results_kdl_vel, "results/kuka_iiwa_vel_kdl.csv");
    toCSV(results_hyrodyn_vel, "results/kuka_iiwa_vel_hyrodyn.csv");
    toCSV(results_kdl_acc, "results/kuka_iiwa_acc_kdl.csv");
    toCSV(results_hyrodyn_acc, "results/kuka_iiwa_acc_hyrodyn.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_vel);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_acc);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_acc);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5SingleLeg("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5SingleLeg("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5SingleLeg("hyrodyn", true);

    const std::string root = "RH5_Root_Link", tip = "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl_vel = evaluateVelocitySceneQuadraticCost(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl_acc = evaluateAccelerationSceneTSID(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl_vel, "results/rh5_single_leg_vel_kdl.csv");
    toCSV(results_hyrodyn_vel, "results/rh5_single_leg_vel_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_vel, "results/rh5_single_leg_vel_hyrodyn_hybrid.csv");
    toCSV(results_kdl_acc, "results/rh5_single_leg_acc_kdl.csv");
    toCSV(results_hyrodyn_acc, "results/rh5_single_leg_acc_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_acc, "results/rh5_single_leg_acc_hyrodyn_hybrid.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_vel);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_acc);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_acc);
    cout << " ----------- Results AccelerationSceneTSID Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_acc);
}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs Model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5Legs("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5Legs("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5Legs("hyrodyn", true);

    const std::string root = "world", tip = "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl_vel = evaluateVelocitySceneQuadraticCost(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl_acc = evaluateAccelerationSceneTSID(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl_vel, "results/rh5_legs_vel_kdl.csv");
    toCSV(results_hyrodyn_vel, "results/rh5_legs_vel_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_vel, "results/rh5_legs_vel_hyrodyn_hybrid.csv");
    toCSV(results_kdl_acc, "results/rh5_legs_acc_kdl.csv");
    toCSV(results_hyrodyn_acc, "results/rh5_legs_acc_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_acc, "results/rh5_legs_acc_hyrodyn_hybrid.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_vel);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_acc);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_acc);
    cout << " ----------- Results AccelerationSceneTSID Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_acc);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5("hyrodyn", true);

    const std::string root = "world", tip = "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl_vel = evaluateVelocitySceneQuadraticCost(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl_acc = evaluateAccelerationSceneTSID(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl_vel, "results/rh5_vel_kdl.csv");
    toCSV(results_hyrodyn_vel, "results/rh5_vel_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_vel, "results/rh5_vel_hyrodyn_hybrid.csv");
    toCSV(results_kdl_acc, "results/rh5_acc_kdl.csv");
    toCSV(results_hyrodyn_acc, "results/rh5_acc_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_acc, "results/rh5_acc_hyrodyn_hybrid.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_vel);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_acc);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_acc);
    cout << " ----------- Results AccelerationSceneTSID Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_acc);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5v2("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5v2("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5v2("hyrodyn", true);

    const std::string root = "RH5v2_Root_Link", tip = "ALWristFT_Link";
    map<string,base::VectorXd> results_kdl_vel = evaluateVelocitySceneQuadraticCost(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_vel = evaluateVelocitySceneQuadraticCost(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl_acc = evaluateAccelerationSceneTSID(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid_acc = evaluateAccelerationSceneTSID(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl_vel, "results/rh5v2_vel_kdl.csv");
    toCSV(results_hyrodyn_vel, "results/rh5v2_vel_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_vel, "results/rh5v2_vel_hyrodyn_hybrid.csv");
    toCSV(results_kdl_acc, "results/rh5v2_acc_kdl.csv");
    toCSV(results_hyrodyn_acc, "results/rh5v2_acc_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid_acc, "results/rh5v2_acc_hyrodyn_hybrid.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_vel);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelKDL) -----------" << endl;
    printResults(results_kdl_acc);
    cout << " ----------- Results AccelerationSceneTSID (RobotModelHyrodyn) -----------" << endl;
    printResults(results_hyrodyn_acc);
    cout << " ----------- Results AccelerationSceneTSID Hybrid (RobotModelHyrodyn Hybrid) -----------" << endl;
    printResults(results_hyrodyn_hybrid_acc);
}

void runBenchmarks(int n_samples){
    boost::filesystem::create_directory("results");

    runKUKAIiwaBenchmarks(n_samples);
    runRH5SingleLegBenchmarks(n_samples);
    runRH5LegsBenchmarks(n_samples);
    runRH5Benchmarks(n_samples);
    runRH5v2Benchmarks(n_samples);
}

int main(){
    srand(time(NULL));
    int n_samples = 100;
    runBenchmarks(n_samples);
}
