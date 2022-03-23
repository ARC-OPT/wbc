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

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean()/1000 << " ms +/- " << stdDev(results["scene_update"]/1000) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean()/1000 << " ms +/- " << stdDev(results["scene_solve"]/1000) << endl;
}

map<string,base::VectorXd> evaluateVelocitySceneQuadraticCost(RobotModelPtr robot_model, const std::string &root, const std::string &tip, int n_samples){
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    (dynamic_pointer_cast<QPOASESSolver>(solver))->setMaxNoWSR(1000);
    qpOASES::Options options;
    options.setToFast();
    options.printLevel = qpOASES::PL_NONE;
    (dynamic_pointer_cast<QPOASESSolver>(solver))->setOptions(options);

    ConstraintConfig cart_constraint("cart_pos_ctrl",0,root,tip,root,1);
    WbcScenePtr scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
    if(!scene->configure({cart_constraint}))
        throw std::runtime_error("Failed to configure VelocitySceneQuadraticCost");
    return evaluateScene(scene, n_samples);
}

map<string,base::VectorXd> evaluateAccelerationSceneTSID(RobotModelPtr robot_model, const std::string &root, const std::string &tip, int n_samples){
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

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;

    const std::string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";

    RobotModelConfig config_kdl("../../../models/kuka/urdf/kuka_iiwa.urdf");
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    RobotModelConfig config_hyrodyn("../../../models/kuka/urdf/kuka_iiwa.urdf");
    config_hyrodyn.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

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

    const std::string root = "RH5_Root_Link", tip = "LLAnkle_FT";

    RobotModelConfig config_kdl("../../../models/rh5/urdf/rh5_single_leg.urdf");
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    RobotModelConfig config_serial_hyrodyn("../../../models/rh5/urdf/rh5_single_leg.urdf");
    config_serial_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_serial_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    RobotModelConfig config_hybrid_hyrodyn("../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf");
    config_hybrid_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hybrid_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

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

    const std::string root = "world", tip = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config_kdl("../../../models/rh5/urdf/rh5_legs.urdf");
    config_kdl.floating_base = true;
    config_kdl.world_frame_id = "world";
    config_kdl.floating_base_state = floating_base_state;
    config_kdl.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config_kdl.contact_points.elements = {1,1};
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    RobotModelConfig config_serial_hyrodyn = config_kdl;
    config_serial_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_serial_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    RobotModelConfig config_hybrid_hyrodyn = config_kdl;
    config_hybrid_hyrodyn.file = "../../../models/rh5/urdf/rh5_legs_hybrid.urdf";
    config_hybrid_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hybrid_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

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

    const std::string root = "world", tip = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    RobotModelConfig config_kdl("../../../models/rh5/urdf/rh5.urdf");
    config_kdl.floating_base = true;
    config_kdl.world_frame_id = "world";
    config_kdl.floating_base_state = floating_base_state;
    config_kdl.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config_kdl.contact_points.elements = {1,1};
    RobotModelPtr robot_model_kdl = std::make_shared<RobotModelKDL>();
    if(!robot_model_kdl->configure(config_kdl))
        throw std::runtime_error("Failed to configure RobotModelKDL");

    RobotModelConfig config_serial_hyrodyn = config_kdl;
    config_serial_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_serial_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    RobotModelConfig config_hybrid_hyrodyn = config_kdl;
    config_hybrid_hyrodyn.file = "../../../models/rh5/urdf/rh5_hybrid.urdf";
    config_hybrid_hyrodyn.submechanism_file = "../../../models/rh5/hyrodyn/rh5_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hybrid_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

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

    const std::string root = "RH5v2_Root_Link", tip = "ALWristFT_Link";

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

    RobotModelConfig config_serial_hyrodyn = config_kdl;
    config_serial_hyrodyn.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
    RobotModelPtr robot_model_hyrodyn = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn->configure(config_serial_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

    RobotModelConfig config_hybrid_hyrodyn("../../../models/rh5v2/urdf/rh5v2_hybrid.urdf");
    config_hybrid_hyrodyn.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    RobotModelPtr robot_model_hyrodyn_hybrid = std::make_shared<RobotModelHyrodyn>();
    if(!robot_model_hyrodyn_hybrid->configure(config_hybrid_hyrodyn))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");

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
