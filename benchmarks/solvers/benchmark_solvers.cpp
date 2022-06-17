#include <iostream>
#include <boost/filesystem.hpp>
#include <core/RobotModelConfig.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <solvers/qpswift/QPSwiftSolver.hpp>
#include "../benchmarks_common.hpp"
#include "../robot_models_common.hpp"

using namespace std;
using namespace wbc;

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean() << " ms +/- " << stdDev(results["scene_update"]) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean() << " ms +/- " << stdDev(results["scene_solve"]) << endl;
}

map<string, base::VectorXd> evaluateQPOases(RobotModelPtr robot_model, string root, string tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

map<string, base::VectorXd> evaluateQPSwift(RobotModelPtr robot_model, string root, string tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPSwiftSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    RobotModelPtr robot_model = makeRobotModelKUKAIiwa("hyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples);

    toCSV(results_qp_oases, "results/kuka_iiwa_qpoases.csv");
    toCSV(results_qp_swift, "results/kuka_iiwa_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    RobotModelPtr robot_model = makeRobotModelRH5SingleLeg("hyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_single_leg_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_single_leg_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs model -----------" << endl;
    RobotModelPtr robot_model = makeRobotModelRH5Legs("hyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_legs_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_legs_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 model -----------" << endl;
    RobotModelPtr robot_model = makeRobotModelRH5("hyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelPtr robot_model = makeRobotModelRH5v2("hyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples);

    toCSV(results_qp_oases, "results/rh5v2_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5v2_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
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
