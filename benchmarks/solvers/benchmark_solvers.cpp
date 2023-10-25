#include <iostream>
#include <boost/filesystem.hpp>
#include <core/RobotModelConfig.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <scenes/AccelerationSceneReducedTSID.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <solvers/qpswift/QPSwiftSolver.hpp>
#include <solvers/proxqp/ProxQPSolver.hpp>
#include <solvers/eiquadprog/EiquadprogSolver.hpp>
#include "../benchmarks_common.hpp"

using namespace std;
using namespace wbc;

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean() << " ms +/- " << stdDev(results["scene_update"]) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean() << " ms +/- " << stdDev(results["scene_solve"]) << endl;
}

map<string, base::VectorXd> evaluateQPOases(RobotModelPtr robot_model, string root, string tip, int n_samples, string type){
    clog << "Evaluate QPOases solver in " << type << " scene" << endl;
    
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();

    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene;
    if(type == "vel")
        scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    else
        scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver);
    if(!scene->configure({cart_task}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneReducedTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

map<string, base::VectorXd> evaluateQPSwift(RobotModelPtr robot_model, string root, string tip, int n_samples, string type){
    clog << "Evaluate QPSwift solver in " << type << " scene" << endl;
    
    QPSolverPtr solver = std::make_shared<QPSwiftSolver>();

    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene;
    if(type == "vel")
        scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    else
        scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver);
    if(!scene->configure({cart_task}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneReducedTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

map<string, base::VectorXd> evaluateEiquadprog(RobotModelPtr robot_model, string root, string tip, int n_samples, string type){
    clog << "Evaluate Eiquadprog solver in " << type << " scene" << endl;
    
    QPSolverPtr solver = std::make_shared<EiquadprogSolver>();

    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene;
    if(type == "vel")
        scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    else
        scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver);
    if(!scene->configure({cart_task}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneReducedTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

map<string, base::VectorXd> evaluateProxQP(RobotModelPtr robot_model, string root, string tip, int n_samples, string type){
    clog << "Evaluate ProxQP solver in " << type << " scene" << endl;
    
    QPSolverPtr solver = std::make_shared<ProxQPSolver>();

    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene;
    if(type == "vel")
        scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    else
        scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver);
    if(!scene->configure({cart_task}))
        throw std::runtime_error("Failed to configure evaluateAccelerationSceneReducedTSID");

    return evaluateWBCSceneRandom(scene, n_samples);
}

void runKUKAIiwaBenchmarks(int n_samples){
    
    RobotModelConfig cfg;
    cfg.file_or_string = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();
    if(!robot_model->configure(cfg))abort();

    base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
    robot_model->update(joint_state);

    clog << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    map<string,base::VectorXd> results_qp_oases_vel   = evaluateQPOases(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "vel");
    map<string,base::VectorXd> results_qp_swift_vel   = evaluateQPSwift(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "vel");
    map<string,base::VectorXd> results_proxqp_vel   = evaluateProxQP(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "vel");
    map<string,base::VectorXd> results_eiquadprog_vel = evaluateEiquadprog(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "vel");
    map<string,base::VectorXd> results_qp_oases_acc   = evaluateQPOases(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "acc");
    map<string,base::VectorXd> results_qp_swift_acc   = evaluateQPSwift(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "acc");
    map<string,base::VectorXd> results_proxqp_acc     = evaluateProxQP(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "acc");
    map<string,base::VectorXd> results_eiquadprog_acc = evaluateEiquadprog(robot_model, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", n_samples, "acc");

    toCSV(results_qp_oases_vel, "results/kuka_iiwa_vel_qpoases.csv");
    toCSV(results_qp_swift_vel, "results/kuka_iiwa_vel_qpswift.csv");
    toCSV(results_proxqp_vel, "results/kuka_iiwa_vel_proxqp.csv");
    toCSV(results_eiquadprog_vel, "results/kuka_iiwa_vel_eiquadprog.csv");
    toCSV(results_qp_oases_acc, "results/kuka_iiwa_acc_qpoases.csv");
    toCSV(results_qp_swift_acc, "results/kuka_iiwa_acc_qpswift.csv");
    toCSV(results_proxqp_acc, "results/kuka_iiwa_acc_proxqp.csv");
    toCSV(results_eiquadprog_acc, "results/kuka_iiwa_acc_eiquadprog.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (QPOases) -----------" << endl;
    printResults(results_qp_oases_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (QPSwift) -----------" << endl;
    printResults(results_qp_swift_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (ProxQP) -----------" << endl;
    printResults(results_proxqp_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_vel);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (ProxQP) -----------" << endl;
    printResults(results_proxqp_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_acc);
}

void runRH5SingleLegBenchmarks(int n_samples){
    
    RobotModelConfig cfg;
    cfg.file_or_string = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();
    if(!robot_model->configure(cfg))abort();

    base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
    base::samples::RigidBodyStateSE3 rbs = randomFloatingBaseState();
    robot_model->update(joint_state, rbs);

    clog << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    map<string,base::VectorXd> results_qp_oases_vel = evaluateQPOases(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_swift_vel = evaluateQPSwift(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_proxqp_vel = evaluateProxQP(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_eiquadprog_vel = evaluateEiquadprog(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_oases_acc = evaluateQPOases(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_qp_swift_acc = evaluateQPSwift(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_proxqp_acc   = evaluateProxQP(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_eiquadprog_acc = evaluateEiquadprog(robot_model, "RH5_Root_Link", "LLAnkle_FT", n_samples, "acc");

    toCSV(results_qp_oases_vel, "results/rh5_single_leg_vel_qpoases.csv");
    toCSV(results_qp_swift_vel, "results/rh5_single_leg_vel_qpswift.csv");
    toCSV(results_proxqp_vel, "results/rh5_single_leg_vel_proxqp.csv");
    toCSV(results_eiquadprog_vel, "results/rh5_single_leg_vel_eiquadprog.csv");
    toCSV(results_qp_oases_acc, "results/rh5_single_leg_acc_qpoases.csv");
    toCSV(results_qp_swift_acc, "results/rh5_single_leg_acc_qpswift.csv");
    toCSV(results_proxqp_acc, "results/rh5_single_leg_acc_proxqp.csv");
    toCSV(results_eiquadprog_acc, "results/rh5_single_leg_acc_eiquadprog.csv");


    cout << " ----------- Results VelocitySceneQuadraticCost (QPOases) -----------" << endl;
    printResults(results_qp_oases_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (QPSwift) -----------" << endl;
    printResults(results_qp_swift_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (ProxQP) -----------" << endl;
    printResults(results_proxqp_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_vel);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (ProxQP) -----------" << endl;
    printResults(results_proxqp_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_acc);
}

void runRH5LegsBenchmarks(int n_samples){
    
    RobotModelConfig cfg;
    cfg.file_or_string = "../../../models/rh5/urdf/rh5_legs.urdf";
    cfg.floating_base = true;
    ActiveContact contact(1,0.6);
    contact.wx = 0.2;
    contact.wy = 0.08;
    cfg.contact_points.names = {"FL_SupportCenter", "FR_SupportCenter"};
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();
    if(!robot_model->configure(cfg))abort();

    base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    joint_state.time = base::Time::now();
    robot_model->update(joint_state, floating_base_state);

    clog << " ----------- Evaluating RH5 Legs model -----------" << endl;
    map<string,base::VectorXd> results_qp_oases_vel = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_swift_vel = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_proxqp_vel = evaluateProxQP(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_eiquadprog_vel = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_oases_acc = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_qp_swift_acc = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_proxqp_acc   = evaluateProxQP(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_eiquadprog_acc = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples, "acc");

    toCSV(results_qp_oases_vel, "results/rh5_legs_vel_qpoases.csv");
    toCSV(results_qp_swift_vel, "results/rh5_legs_vel_qpswift.csv");
    toCSV(results_proxqp_vel, "results/rh5_legs_vel_proxqp.csv");
    toCSV(results_eiquadprog_vel, "results/rh5_legs_vel_eiquadprog.csv");
    toCSV(results_qp_oases_acc, "results/rh5_legs_acc_qpoases.csv");
    toCSV(results_qp_swift_acc, "results/rh5_legs_acc_qpswift.csv");
    toCSV(results_proxqp_acc, "results/rh5_legs_acc_proxqp.csv");
    toCSV(results_eiquadprog_acc, "results/rh5_legs_acc_eiquadprog.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (QPOases) -----------" << endl;
    printResults(results_qp_oases_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (QPSwift) -----------" << endl;
    printResults(results_qp_swift_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (ProxQP) -----------" << endl;
    printResults(results_proxqp_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_vel);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (ProxQP) -----------" << endl;
    printResults(results_proxqp_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_acc);
}

void runRH5Benchmarks(int n_samples){
    
    RobotModelConfig cfg;
    cfg.file_or_string = "../../../models/rh5/urdf/rh5.urdf";
    cfg.floating_base = true;
    ActiveContact contact(1,0.6);
    contact.wx = 0.2;
    contact.wy = 0.08;
    cfg.contact_points.names = {"FL_SupportCenter", "FR_SupportCenter"};
    cfg.contact_points.elements = {contact, contact};
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();
    if(!robot_model->configure(cfg))abort();
    base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(0.0, 0.0, 0.89);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    joint_state.time = base::Time::now();
    robot_model->update(joint_state, floating_base_state);

    clog << " ----------- Evaluating RH5 model -----------" << endl;
    map<string,base::VectorXd> results_qp_oases_vel = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_swift_vel = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_proxqp_vel = evaluateProxQP(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_eiquadprog_vel = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples, "vel");
    map<string,base::VectorXd> results_qp_oases_acc = evaluateQPOases(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_qp_swift_acc = evaluateQPSwift(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_proxqp_acc   = evaluateProxQP(robot_model, "world", "LLAnkle_FT", n_samples, "acc");
    map<string,base::VectorXd> results_eiquadprog_acc = evaluateEiquadprog(robot_model, "world", "LLAnkle_FT", n_samples, "acc");

    toCSV(results_qp_oases_vel, "results/rh5_vel_qpoases.csv");
    toCSV(results_qp_swift_vel, "results/rh5_vel_qpswift.csv");
    toCSV(results_proxqp_vel, "results/rh5_vel_proxqp.csv");
    toCSV(results_eiquadprog_vel, "results/rh5_vel_eiquadprog.csv");
    toCSV(results_qp_oases_acc, "results/rh5_acc_qpoases.csv");
    toCSV(results_qp_swift_acc, "results/rh5_acc_qpswift.csv");
    toCSV(results_proxqp_acc, "results/rh5_acc_proxqp.csv");
    toCSV(results_eiquadprog_acc, "results/rh5_acc_eiquadprog.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (QPOases) -----------" << endl;
    printResults(results_qp_oases_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (QPSwift) -----------" << endl;
    printResults(results_qp_swift_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (ProxQP) -----------" << endl;
    printResults(results_proxqp_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_vel);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (ProxQP) -----------" << endl;
    printResults(results_proxqp_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_acc);
}

void runRH5v2Benchmarks(int n_samples){

    RobotModelConfig cfg;
    cfg.file_or_string = "../../../models/rh5v2/urdf/rh5v2.urdf";
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();
    if(!robot_model->configure(cfg))abort();

    base::samples::Joints joint_state = randomJointState(robot_model->jointLimits());
    base::samples::RigidBodyStateSE3 rbs = randomFloatingBaseState();
    robot_model->update(joint_state, rbs);

    clog << " ----------- Evaluating RH5v2 model -----------" << endl;
    map<string,base::VectorXd> results_qp_oases_vel = evaluateQPOases(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "vel");
    map<string,base::VectorXd> results_qp_swift_vel = evaluateQPSwift(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "vel");
    map<string,base::VectorXd> results_proxqp_vel = evaluateProxQP(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "vel");
    map<string,base::VectorXd> results_eiquadprog_vel = evaluateEiquadprog(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "vel");
    map<string,base::VectorXd> results_qp_oases_acc = evaluateQPOases(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "acc");
    map<string,base::VectorXd> results_qp_swift_acc = evaluateQPSwift(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "acc");
    map<string,base::VectorXd> results_proxqp_acc   = evaluateProxQP(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "acc");
    map<string,base::VectorXd> results_eiquadprog_acc = evaluateEiquadprog(robot_model, "RH5v2_Root_Link", "ALWristFT_Link", n_samples, "acc");

    toCSV(results_qp_oases_vel, "results/rh5v2_vel_qpoases.csv");
    toCSV(results_qp_swift_vel, "results/rh5v2_vel_qpswift.csv");
    toCSV(results_proxqp_vel, "results/rh5v2_vel_proxqp.csv");
    toCSV(results_eiquadprog_vel, "results/rh5v2_vel_eiquadprog.csv");
    toCSV(results_qp_oases_acc, "results/rh5v2_acc_qpoases.csv");
    toCSV(results_qp_swift_acc, "results/rh5v2_acc_qpswift.csv");
    toCSV(results_proxqp_acc, "results/rh5v2_acc_proxqp.csv");
    toCSV(results_eiquadprog_acc, "results/rh5v2_acc_eiquadprog.csv");

    cout << " ----------- Results VelocitySceneQuadraticCost (QPOases) -----------" << endl;
    printResults(results_qp_oases_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (QPSwift) -----------" << endl;
    printResults(results_qp_swift_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (ProxQP) -----------" << endl;
    printResults(results_proxqp_vel);
    cout << " ----------- Results VelocitySceneQuadraticCost (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_vel);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPOases) -----------" << endl;
    printResults(results_qp_oases_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (QPSwift) -----------" << endl;
    printResults(results_qp_swift_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (ProxQP) -----------" << endl;
    printResults(results_proxqp_acc);
    cout << " ----------- Results AccelerationSceneReducedTSID (Eiquadprog) -----------" << endl;
    printResults(results_eiquadprog_acc);
}

void runBenchmarks(int n_samples){
    boost::filesystem::create_directory("results");

    runRH5Benchmarks(n_samples);
    runRH5v2Benchmarks(n_samples);
    runKUKAIiwaBenchmarks(n_samples);
    runRH5SingleLegBenchmarks(n_samples);
    runRH5LegsBenchmarks(n_samples);
}

int main(){
    srand(time(NULL));
    int n_samples = 1000;
    runBenchmarks(n_samples);
}
