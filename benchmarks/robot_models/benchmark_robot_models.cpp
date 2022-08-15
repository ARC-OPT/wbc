#include <boost/filesystem.hpp>
#include "../benchmarks_common.hpp"
#include "core/RobotModelFactory.hpp"
#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/rbdl/RobotModelRBDL.hpp"
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"

using namespace wbc;
using namespace std;

base::VectorXd evalBodyJacobian(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->bodyJacobian(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalSpaceJacobian(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->spaceJacobian(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalFK(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->rigidBodyState(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalBiasForces(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->biasForces();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalJointSpaceInertiaMat(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->jointSpaceInertiaMatrix();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalSpatialAccelerationBias(RobotModelPtr robot_model,  const string &root, const string &tip, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->spatialAccelerationBias(root, tip);
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
    }
    return results;
}

base::VectorXd evalCoM(RobotModelPtr robot_model, int n_samples){
    base::VectorXd results(n_samples);
    for(int i = 0; i < n_samples; i++){
        base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(robot_model->floatingBaseState());
        robot_model->update(joint_state, floating_base_state);
        base::Time start = base::Time::now();
        robot_model->centerOfMass();
        results[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;
        usleep(0.001*1e6);
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

    RobotModelConfig cfg;
    cfg.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    cfg.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";

    RobotModelPtr robot_model_kdl =  std::make_shared<RobotModelKDL>();
    RobotModelPtr robot_model_hyrodyn =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_rbdl =  std::make_shared<RobotModelRBDL>();
    RobotModelPtr robot_model_pinocchio =  std::make_shared<RobotModelPinocchio>();

    if(!robot_model_kdl->configure(cfg)) abort();
    if(!robot_model_hyrodyn->configure(cfg))abort();
    if(!robot_model_pinocchio->configure(cfg))abort();
    if(!robot_model_rbdl->configure(cfg))abort();

    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_pinocchio = evaluateRobotModel(robot_model_pinocchio, root, tip, n_samples);
    map<string,base::VectorXd> results_rbdl = evaluateRobotModel(robot_model_rbdl, root, tip, n_samples);

    toCSV(results_kdl, "results/kuka_iiwa_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/kuka_iiwa_robot_model_hyrodyn.csv");
    toCSV(results_pinocchio, "results/kuka_iiwa_robot_model_pinocchio.csv");
    toCSV(results_rbdl, "results/kuka_iiwa_robot_model_rbdl.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelPinocchio -----------" << endl;
    printResults(results_pinocchio);
    cout << " ----------- Results RobotModelRBDL -----------" << endl;
    printResults(results_rbdl);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";

    RobotModelPtr robot_model_kdl =  std::make_shared<RobotModelKDL>();
    RobotModelPtr robot_model_hyrodyn =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_hyrodyn_hybrid =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_rbdl =  std::make_shared<RobotModelRBDL>();
    RobotModelPtr robot_model_pinocchio =  std::make_shared<RobotModelPinocchio>();

    if(!robot_model_kdl->configure(cfg)) abort();
    if(!robot_model_hyrodyn->configure(cfg))abort();
    if(!robot_model_pinocchio->configure(cfg))abort();
    if(!robot_model_rbdl->configure(cfg))abort();
    cfg.file = "../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    if(!robot_model_hyrodyn_hybrid->configure(cfg))abort();

    const std::string root = "RH5_Root_Link", tip = "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_pinocchio = evaluateRobotModel(robot_model_pinocchio, root, tip, n_samples);
    map<string,base::VectorXd> results_rbdl = evaluateRobotModel(robot_model_rbdl, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_single_leg_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_single_leg_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_single_leg_robot_model_hyrodyn_hybrid.csv");    
    toCSV(results_pinocchio, "results/rh5_single_leg_robot_model_pinocchio.csv");
    toCSV(results_rbdl, "results/rh5_single_leg_robot_model_rbdl.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
    cout << " ----------- Results RobotModelPinocchio -----------" << endl;
    printResults(results_pinocchio);
    cout << " ----------- Results RobotModelRBDL -----------" << endl;
    printResults(results_rbdl);
}


void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
    cfg.floating_base = true;

    RobotModelPtr robot_model_kdl =  std::make_shared<RobotModelKDL>();
    RobotModelPtr robot_model_hyrodyn =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_hyrodyn_hybrid =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_rbdl =  std::make_shared<RobotModelRBDL>();
    RobotModelPtr robot_model_pinocchio =  std::make_shared<RobotModelPinocchio>();

    if(!robot_model_kdl->configure(cfg)) abort();
    if(!robot_model_hyrodyn->configure(cfg))abort();
    if(!robot_model_pinocchio->configure(cfg))abort();
    if(!robot_model_rbdl->configure(cfg))abort();
    cfg.file = "../../../models/rh5/urdf/rh5_legs_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs_hybrid.yml";
    if(!robot_model_hyrodyn_hybrid->configure(cfg))abort();

    const string root = "world", tip =  "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_pinocchio = evaluateRobotModel(robot_model_pinocchio, root, tip, n_samples);
    map<string,base::VectorXd> results_rbdl = evaluateRobotModel(robot_model_rbdl, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_legs_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_legs_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_legs_robot_model_hyrodyn_hybrid.csv");
    toCSV(results_pinocchio, "results/rh5_legs_robot_model_pinocchio.csv");
    toCSV(results_rbdl, "results/rh5_legs_robot_model_rbdl.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
    cout << " ----------- Results RobotModelPinocchio -----------" << endl;
    printResults(results_pinocchio);
    cout << " ----------- Results RobotModelRBDL -----------" << endl;
    printResults(results_rbdl);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    cfg.floating_base = true;

    RobotModelPtr robot_model_kdl =  std::make_shared<RobotModelKDL>();
    RobotModelPtr robot_model_hyrodyn =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_hyrodyn_hybrid =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_rbdl =  std::make_shared<RobotModelRBDL>();
    RobotModelPtr robot_model_pinocchio =  std::make_shared<RobotModelPinocchio>();

    if(!robot_model_kdl->configure(cfg)) abort();
    if(!robot_model_hyrodyn->configure(cfg))abort();
    if(!robot_model_pinocchio->configure(cfg))abort();
    if(!robot_model_rbdl->configure(cfg))abort();
    cfg.file = "../../../models/rh5/urdf/rh5_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_hybrid.yml";
    if(!robot_model_hyrodyn_hybrid->configure(cfg))abort();

    const string root = "world", tip =  "LLAnkle_FT";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_pinocchio = evaluateRobotModel(robot_model_pinocchio, root, tip, n_samples);
    map<string,base::VectorXd> results_rbdl = evaluateRobotModel(robot_model_rbdl, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5_robot_model_hyrodyn_hybrid.csv");
    toCSV(results_pinocchio, "results/rh5_robot_model_pinocchio.csv");
    toCSV(results_rbdl, "results/rh5_robot_model_rbdl.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
    cout << " ----------- Results RobotModelPinocchio -----------" << endl;
    printResults(results_pinocchio);
    cout << " ----------- Results RobotModelRBDL -----------" << endl;
    printResults(results_rbdl);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5v2/urdf/rh5v2.urdf";
    cfg.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";

    RobotModelPtr robot_model_kdl =  std::make_shared<RobotModelKDL>();
    RobotModelPtr robot_model_hyrodyn =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_hyrodyn_hybrid =  std::make_shared<RobotModelHyrodyn>();
    RobotModelPtr robot_model_rbdl =  std::make_shared<RobotModelRBDL>();
    RobotModelPtr robot_model_pinocchio =  std::make_shared<RobotModelPinocchio>();

    if(!robot_model_kdl->configure(cfg)) abort();
    if(!robot_model_hyrodyn->configure(cfg))abort();
    if(!robot_model_pinocchio->configure(cfg))abort();
    if(!robot_model_rbdl->configure(cfg))abort();
    cfg.file = "../../../models/rh5v2/urdf/rh5v2_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    if(!robot_model_hyrodyn_hybrid->configure(cfg))abort();

    const std::string root = "RH5v2_Root_Link", tip = "ALWristFT_Link";
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
    map<string,base::VectorXd> results_pinocchio = evaluateRobotModel(robot_model_pinocchio, root, tip, n_samples);
    map<string,base::VectorXd> results_rbdl = evaluateRobotModel(robot_model_rbdl, root, tip, n_samples);

    toCSV(results_kdl, "results/rh5v2_robot_model_kdl.csv");
    toCSV(results_hyrodyn, "results/rh5v2_robot_model_hyrodyn.csv");
    toCSV(results_hyrodyn_hybrid, "results/rh5v2_robot_model_hyrodyn_hybrid.csv");
    toCSV(results_pinocchio, "results/rh5v2_robot_model_pinocchio.csv");
    toCSV(results_rbdl, "results/rh5v2_robot_model_rbdl.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
    cout << " ----------- Results RobotModelHyrodyn Hybrid -----------" << endl;
    printResults(results_hyrodyn_hybrid);
    cout << " ----------- Results RobotModelPinocchio -----------" << endl;
    printResults(results_pinocchio);
    cout << " ----------- Results RobotModelRBDL -----------" << endl;
    printResults(results_rbdl);
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
    int n_samples = 1000;
    runBenchmarks(n_samples);
}
