#include <core/RobotModelConfig.hpp>
#include <robot_models/kdl/RobotModelKDL.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <scenes/VelocityScene.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/hls/HierarchicalLSSolver.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <boost/filesystem.hpp>
#include "../benchmarks_common.hpp"

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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
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
        results[i] = (double)(base::Time::now()-start).toMicroseconds();
        usleep(0.01*1e6);
    }
    return results;
}

void printResults(map<string,base::VectorXd> results){
    cout << "Space Jacobian     " << results["space_jac"].mean()/1000 << " ms +/- " << stdDev(results["space_jac"]/1000) << endl;
    cout << "Body Jacobian      " << results["body_jac"].mean()/1000 << " ms +/- " << stdDev(results["body_jac"]/1000) << endl;
    cout << "Forward Kinematics " << results["FK"].mean()/1000 << " ms +/- " << stdDev(results["FK"]/1000) << endl;
    cout << "Jnt Inertia Mat    " << results["joint_space_inertia_mat"].mean()/1000 << " ms +/- " << stdDev(results["joint_space_inertia_mat"]/1000) << endl;
    cout << "Bias Forces        " << results["bias_forces"].mean()/1000 << " ms +/- " << stdDev(results["bias_forces"]/1000) << endl;
    cout << "Spatial Acc Bias   " << results["spatial_acc_bias"].mean()/1000 << " ms +/- " << stdDev(results["spatial_acc_bias"]/1000) << endl;
    cout << "CoM                " << results["com"].mean()/1000 << " ms +/- " << stdDev(results["com"]/1000) << endl;
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
    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";

    RobotModelConfig config_kdl("../../../models/kuka/urdf/kuka_iiwa.urdf");
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    toCSV(results_kdl, "results/kuka_iiwa_robot_model_kdl.csv");

    RobotModelConfig config_hyrodyn("../../../models/kuka/urdf/kuka_iiwa.urdf");
    config_hyrodyn.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    toCSV(results_hyrodyn, "results/kuka_iiwa_robot_model_hyrodyn.csv");

    cout << " ----------- Results RobotModelKDL -----------" << endl;
    printResults(results_kdl);
    cout << " ----------- Results RobotModelHyrodyn -----------" << endl;
    printResults(results_hyrodyn);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    const std::string root = "RH5_Root_Link", tip = "LLAnkle_FT";
    RobotModelConfig config_serial_kdl("../../../models/rh5/urdf/rh5_single_leg.urdf");
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_serial_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    toCSV(results_kdl, "results/rh5_single_leg_robot_model_kdl.csv");

    RobotModelConfig config_serial_hyrodyn("../../../models/rh5/urdf/rh5_single_leg.urdf");
    config_serial_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_serial_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    toCSV(results_hyrodyn, "results/rh5_single_leg_robot_model_hyrodyn.csv");

    RobotModelConfig config_hybrid_hyrodyn("../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf");
    config_hybrid_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hybrid_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
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
    const string root = "world", tip =  "LLAnkle_FT";
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config_kdl;
    config_kdl.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    config_kdl.floating_base = true;
    config_kdl.world_frame_id = "world";
    config_kdl.floating_base_state = floating_base_state;
    config_kdl.contact_points = {"LLAnkle_FT", "LRAnkle_FT"};
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    toCSV(results_kdl, "results/rh5_legs_robot_model_kdl.csv");

    RobotModelConfig config_hyrodyn = config_kdl;
    config_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    toCSV(results_hyrodyn, "results/rh5_legs_robot_model_hyrodyn.csv");

    RobotModelConfig config_hyrodyn_hybrid = config_kdl;
    config_hyrodyn_hybrid.file = "../../../models/rh5/urdf/rh5_legs_hybrid.urdf";
    config_hyrodyn_hybrid.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hyrodyn_hybrid))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
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
    const string root = "world", tip =  "LLAnkle_FT";
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config_kdl;
    config_kdl.file = "../../../models/rh5/urdf/rh5.urdf";
    config_kdl.floating_base = true;
    config_kdl.world_frame_id = "world";
    config_kdl.floating_base_state = floating_base_state;
    config_kdl.contact_points = {"LLAnkle_FT", "LRAnkle_FT"};
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    toCSV(results_kdl, "results/rh5_robot_model_kdl.csv");

    RobotModelConfig config_hyrodyn = config_kdl;
    config_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    toCSV(results_hyrodyn, "results/rh5_robot_model_hyrodyn.csv");

    RobotModelConfig config_hyrodyn_hybrid = config_kdl;
    config_hyrodyn_hybrid.file = "../../../models/rh5/urdf/rh5_hybrid.urdf";
    config_hyrodyn_hybrid.submechanism_file = "../../../models/rh5/hyrodyn/rh5_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hyrodyn_hybrid))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
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
    const string root = "RH5v2_Root_Link", tip =  "ALWristFT_Link";
    RobotModelConfig config_kdl("../../../models/rh5v2/urdf/rh5v2.urdf");
    config_kdl.joint_blacklist = {"HeadPitch", "HeadRoll", "HeadYaw",
                              "GLF1Gear", "GLF1ProximalSegment", "GLF1TopSegment",
                              "GLF2Gear", "GLF2ProximalSegment", "GLF2TopSegment",
                              "GLF3Gear", "GLF3ProximalSegment", "GLF3TopSegment",
                              "GLF4Gear", "GLF4ProximalSegment", "GLF4TopSegment", "GLThumb",
                              "GRF1Gear", "GRF1ProximalSegment", "GRF1TopSegment",
                              "GRF2Gear", "GRF2ProximalSegment", "GRF2TopSegment"};
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");
    map<string,base::VectorXd> results_kdl = evaluateRobotModel(robot_model_kdl, root, tip, n_samples);
    toCSV(results_kdl, "results/rh5v2_robot_model_kdl.csv");

    RobotModelConfig config_hyrodyn = config_kdl;
    config_hyrodyn.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn = evaluateRobotModel(robot_model_hyrodyn, root, tip, n_samples);
    toCSV(results_hyrodyn, "results/rh5v2_robot_model_hyrodyn.csv");

    RobotModelConfig config_hyrodyn_hybrid("../../../models/rh5v2/urdf/rh5v2_hybrid.urdf");
    config_hyrodyn_hybrid.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hyrodyn_hybrid))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    map<string,base::VectorXd> results_hyrodyn_hybrid = evaluateRobotModel(robot_model_hyrodyn_hybrid, root, tip, n_samples);
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
