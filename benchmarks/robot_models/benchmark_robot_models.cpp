#include <boost/filesystem.hpp>
#include "../benchmarks_common.hpp"
#include "../robot_models_common.hpp"

using namespace wbc;
using namespace std;

base::VectorXd evalBodyJacobian(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->bodyJacobian(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalSpaceJacobian(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->spaceJacobian(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalFK(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->rigidBodyState(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalBiasForces(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->biasForces();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalJointSpaceInertiaMat(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->jointSpaceInertiaMatrix();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalSpatialAccelerationBias(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->spatialAccelerationBias(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

base::VectorXd evalCoM(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->independentJointNames(), robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->getRobotModelConfig().floating_base_state);
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->centerOfMass();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.01*1e6);
    }
    return results;
}

void printResults(map<string,base::VectorXd> results){
    cout << "Space Jacobian     " << results["space_jac"].mean() << " ms +/- " << stdDev(results["space_jac"]) << endl;
    cout << "Body Jacobian      " << results["body_jac"].mean() << " ms +/- " << stdDev(results["body_jac"]) << endl;
    cout << "Forward Kinematics " << results["FK"].mean() << " ms +/- " << stdDev(results["FK"]) << endl;
    cout << "Jnt Inertia Mat    " << results["joint_space_inertia_mat"].mean() << " ms +/- " << stdDev(results["joint_space_inertia_mat"]) << endl;
    cout << "Bias Forces        " << results["bias_forces"].mean() << " ms +/- " << stdDev(results["bias_forces"]) << endl;
    cout << "Spatial Acc Bias   " << results["spatial_acc_bias"].mean() << " ms +/- " << stdDev(results["spatial_acc_bias"]) << endl;
    cout << "CoM                " << results["com"].mean() << " ms +/- " << stdDev(results["com"]) << endl;
}

map<string,base::VectorXd> evaluateRobotModel(RobotModelPtr robot_model, const string &root, const string &tip, int n_samples){
    map<string,base::VectorXd> results;
    results["space_jac"]               = evalSpaceJacobian(robot_model, root, tip, n_samples);
    results["body_jac"]                = evalBodyJacobian(robot_model, root, tip, n_samples);
    results["FK"]                      = evalFK(robot_model, root, tip, n_samples);
    results["joint_space_inertia_mat"] = evalJointSpaceInertiaMat(robot_model, n_samples);
    results["bias_forces"]             = evalBiasForces(robot_model, n_samples);
    results["spatial_acc_bias"]        = evalSpatialAccelerationBias(robot_model, root, tip, n_samples);
    results["com"]                     = evalCoM(robot_model, n_samples);
    return results;
}

void runKUKAIiwaBenchmarks(int n_samples){    
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelKUKAIiwa("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelKUKAIiwa("hyrodyn");

    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);

    toCSV(results_kdl, "results/kuka_iiwa_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/kuka_iiwa_robot_model_hyrodyn.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;    
    RobotModelPtr robot_model_kdl = makeRobotModelRH5SingleLeg("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5SingleLeg("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5SingleLeg("hyrodyn", true);

    const std::string root = "RH5_Root_Link", tip = "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_single_leg_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_single_leg_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_single_leg_robot_model_hyrodyn_hybrid.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5Legs("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5Legs("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5Legs("hyrodyn", true);

    const string root = "world", tip =  "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_legs_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_legs_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_legs_robot_model_hyrodyn_hybrid.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5("hyrodyn", true);

    const string root = "world", tip =  "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_robot_model_hyrodyn_hybrid.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelPtr robot_model_kdl = makeRobotModelRH5v2("kdl");
    RobotModelPtr robot_model_hyrodyn = makeRobotModelRH5v2("hyrodyn");
    RobotModelPtr robot_model_hyrodyn_hybrid = makeRobotModelRH5v2("hyrodyn", true);

    const std::string root = "RH5v2_Root_Link", tip = "ALWristFT_Link";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5v2_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5v2_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5v2_robot_model_hyrodyn_hybrid.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
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
