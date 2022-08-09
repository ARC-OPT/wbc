#include <iostream>
#include <boost/filesystem.hpp>
#include <core/RobotModelConfig.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <solvers/qpswift/QPSwiftSolver.hpp>
#include <solvers/eiquadprog/EiquadprogSolver.hpp>
#include "../benchmarks_common.hpp"

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

map<string, base::VectorXd> evaluateEiquadprog(RobotModelPtr robot_model, string root, string tip, int n_samples){
    QPSolverPtr solver = std::make_shared<EiquadprogSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    cfg.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(cfg))abort();

    map<string,base::VectorXd> results_qp_oases   = evaluateQPOases(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples);
    map<string,base::VectorXd> results_qp_swift   = evaluateQPSwift(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples);
    map<string,base::VectorXd> results_eiquadprog = evaluateEiquadprog(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples);

    toCSV(results_qp_oases, "results/kuka_iiwa_qpoases.csv");
    toCSV(results_qp_swift, "results/kuka_iiwa_qpswift.csv");
    toCSV(results_eiquadprog, "results/kuka_iiwa_eiquadprog.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
    cout << " ----------- Results AccelerationSceneTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(cfg))abort();

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_eiquadprog = evaluateEiquadprog(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_single_leg_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_single_leg_qpswift.csv");
    toCSV(results_eiquadprog, "results/kuka_iiwa_eiquadprog.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
    cout << " ----------- Results AccelerationSceneTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog);
}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
    cfg.floating_base = true;
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(cfg))abort();

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_eiquadprog = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_legs_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_legs_qpswift.csv");
    toCSV(results_eiquadprog, "results/kuka_iiwa_eiquadprog.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
    cout << " ----------- Results AccelerationSceneTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    cfg.floating_base = true;
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(cfg))abort();

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples);
    map<string,base::VectorXd> results_eiquadprog = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples);

    toCSV(results_qp_oases, "results/rh5_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_qpswift.csv");
    toCSV(results_eiquadprog, "results/kuka_iiwa_eiquadprog.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
    cout << " ----------- Results AccelerationSceneTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5v2/urdf/rh5v2.urdf";
    cfg.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(cfg))abort();

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples);
    map<string,base::VectorXd> results_eiquadprog = evaluateEiquadprog(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples);


    toCSV(results_qp_oases, "results/rh5v2_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5v2_qpswift.csv");
    toCSV(results_eiquadprog, "results/kuka_iiwa_eiquadprog.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);
    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);
    cout << " ----------- Results AccelerationSceneTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog);
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
