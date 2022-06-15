#include <iostream>
#include <boost/filesystem.hpp>
#include <core/RobotModelConfig.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <solvers/qpswift/QPSwiftSolver.hpp>
#include "../benchmarks_common.hpp"

using namespace std;
using namespace wbc;

base::samples::RigidBodyStateSE3 randomConstraintReference(){
    base::samples::RigidBodyStateSE3 ref;
    for(int i = 0; i < 3; i++){
        ref.twist.linear[i] = whiteNoise(1e-4);
        ref.twist.angular[i] = whiteNoise(1e-4);
        ref.acceleration.linear[i] = whiteNoise(1e-4);
        ref.acceleration.angular[i] = whiteNoise(1e-4);
    }
    ref.time = base::Time::now();
    return ref;
}

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean()/1000 << " ms +/- " << stdDev(results["scene_update"]/1000) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean()/1000 << " ms +/- " << stdDev(results["scene_solve"]/1000) << endl;
}

map<string, base::VectorXd> evaluateScene(WbcScenePtr scene, int n_samples){

    base::VectorXd time_scene_update(n_samples);
    base::VectorXd time_scene_solve(n_samples);
    for(int i = 0; i < n_samples; i++){
        for(auto w : scene->getWbcConfig())
            scene->setReference(w.name, randomConstraintReference());

        base::samples::Joints joint_state = randomJointState(scene->getRobotModel()->independentJointNames(), scene->getRobotModel()->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(scene->getRobotModel()->getRobotModelConfig().floating_base_state);
        scene->getRobotModel()->update(joint_state, floating_base_state);

        base::Time start = base::Time::now();
        HierarchicalQP qp = scene->update();
        time_scene_update[i] = (double)(base::Time::now()-start).toMicroseconds();

        try{
            start = base::Time::now();
            scene->solve(qp);
            time_scene_solve[i] = (double)(base::Time::now()-start).toMicroseconds();
        }
        catch(exception e){
            cout<<"Exception"<<endl;
            i--;
            usleep(0.01*1e6);
            scene->getSolver()->reset();
            continue;
        }
        usleep(0.01*1e6);
    }

    map<string, base::VectorXd> results;
    results["scene_update"] = time_scene_update;
    results["scene_solve"]  = time_scene_solve;
    return results;
}

map<string, base::VectorXd> evaluateQPOases(RobotModelPtr robot_model, string root, string tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    (dynamic_pointer_cast<QPOASESSolver>(solver))->setMaxNoWSR(1000);
    qpOASES::Options options;
    options.setToFast();
    options.printLevel = qpOASES::PL_NONE;
    (dynamic_pointer_cast<QPOASESSolver>(solver))->setOptions(options);

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");

    return evaluateScene(scene, n_samples);
}

map<string, base::VectorXd> evaluateQPSwift(RobotModelPtr robot_model, string root, string tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPSwiftSolver>();

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneTSID");

    return evaluateScene(scene, n_samples);
}

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;

    RobotModelConfig config;
    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";
    config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, root, tip, n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, root, tip, n_samples);

    toCSV(results_qp_oases, "results/kuka_iiwa_qpoases.csv");
    toCSV(results_qp_swift, "results/kuka_iiwa_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);

    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);

}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;

}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs model -----------" << endl;

}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 model -----------" << endl;

    RobotModelConfig config;
    const std::string root = "world", tip = "LLAnkle_FT";
    config.file = "../../../models/rh5/urdf/rh5.urdf";
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    config.floating_base = true;
    config.world_frame_id = "world";
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    config.floating_base_state = floating_base_state;
    config.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config.contact_points.elements = {1,1};
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    map<string,base::VectorXd> results_qp_oases = evaluateQPOases(robot_model, root, tip, n_samples);
    map<string,base::VectorXd> results_qp_swift = evaluateQPSwift(robot_model, root, tip, n_samples);

    toCSV(results_qp_oases, "results/rh5_qpoases.csv");
    toCSV(results_qp_swift, "results/rh5_qpswift.csv");

    cout << " ----------- Results AccelerationSceneTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases);

    cout << " ----------- Results AccelerationSceneTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift);

}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
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
