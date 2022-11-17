#include <scenes/VelocityScene.hpp>
#include <scenes/VelocitySceneQuadraticCost.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <solvers/eiquadprog/EiquadprogSolver.hpp>
#include <solvers/qpswift/QPSwiftSolver.hpp>
#include <robot_models/kdl/RobotModelKDL.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <robot_models/rbdl/RobotModelRBDL.hpp>
#include <boost/filesystem.hpp>
#include "../benchmarks_common.hpp"

using namespace wbc;
using namespace std;

void printResults(map<string,base::VectorXd> results){
    cout << "Scene Update     " << results["scene_update"].mean() << " ms +/- " << stdDev(results["scene_update"]) << endl;
    cout << "Scene Solve      " << results["scene_solve"].mean() << " ms +/- " << stdDev(results["scene_solve"]) << endl;
}

void evaluateVelocitySceneQuadraticCost(map<string,RobotModelPtr> robot_models, map<string,QPSolverPtr> solvers,
                                        const string &root, const string &tip,
                                        int n_samples, string robot_name){
    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    for(auto it : robot_models){
        for(auto itt : solvers){
            WbcScenePtr scene = make_shared<VelocitySceneQuadraticCost>(it.second, itt.second);
            if(!scene->configure({cart_task}))
                throw runtime_error("Failed to configure VelocitySceneQuadraticCost");
            map<string,base::VectorXd> results = evaluateWBCSceneRandom(scene, n_samples);
            string filename = "results/" + robot_name + "_vel_" + it.first + "_" + itt.first + ".csv";
            toCSV(results, filename);
            cout << " ----------- Results VelocitySceneQuadraticCost (" + it.first + "," + itt.first + ") -----------" << endl;
            printResults(results);
        }
    }
}

void evaluateAccelerationSceneTSID(map<string,RobotModelPtr> robot_models, map<string,QPSolverPtr> solvers,
                                   const string &root, const string &tip,
                                   int n_samples, string robot_name){
    TaskConfig cart_task("cart_pos_ctrl",0,root,tip,root,1);
    for(auto it : robot_models){
        for(auto itt : solvers){
            WbcScenePtr scene = make_shared<AccelerationSceneTSID>(it.second, itt.second);
            if(!scene->configure({cart_task}))
                throw runtime_error("Failed to configure AccelerationSceneTSID");
            map<string,base::VectorXd> results = evaluateWBCSceneRandom(scene, n_samples);
            toCSV(results, "results/" + robot_name + "_acc_" + it.first + "_" + itt.first + ".csv");
            cout << " ----------- Results AccelerationSceneTSID (" + it.first + "," + itt.first + ") -----------" << endl;
            printResults(results);
        }
    }
}

void runKUKAIiwaBenchmarks(int n_samples){
    cout << " ----------- Evaluating KUKA iiwa model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    cfg.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    const string root = "kuka_lbr_l_link_0", tip = "kuka_lbr_l_tcp";
    const string robot = "kuka_iiwa";

    map<string,RobotModelPtr> robot_models;
    robot_models["kdl"] =  make_shared<RobotModelKDL>();
    robot_models["hyrodyn"] =  make_shared<RobotModelHyrodyn>();
    robot_models["pinocchio"] =  make_shared<RobotModelPinocchio>();
    robot_models["rbdl"] =  make_shared<RobotModelRBDL>();
    for(auto it : robot_models){
        if(!it.second->configure(cfg))
            abort();
        base::samples::Joints joint_state = randomJointState(it.second->jointLimits());
        it.second->update(joint_state);
    }

    map<string,QPSolverPtr> solvers;
    solvers["qpoases"] = make_shared<QPOASESSolver>();
    solvers["eiquadprog"] = make_shared<EiquadprogSolver>();
    solvers["qpswift"] = make_shared<QPSwiftSolver>();

    evaluateVelocitySceneQuadraticCost(robot_models, solvers, root, tip, n_samples, robot);
    evaluateAccelerationSceneTSID(robot_models, solvers, root, tip, n_samples, robot);
}

void runRH5SingleLegBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Single Leg model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    const string robot = "rh5_single_leg";

    map<string,RobotModelPtr> robot_models;
    robot_models["kdl"] =  make_shared<RobotModelKDL>();
    robot_models["hyrodyn"] =  make_shared<RobotModelHyrodyn>();
    robot_models["pinocchio"] =  make_shared<RobotModelPinocchio>();
    robot_models["rbdl"] =  make_shared<RobotModelRBDL>();
    for(auto it : robot_models){
        if(!it.second->configure(cfg))
            abort();
    }
    robot_models["hyrodyn_hybrid"] =  make_shared<RobotModelHyrodyn>();
    cfg.file = "../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    if(!robot_models["hyrodyn_hybrid"]->configure(cfg)) abort();
    for(auto it : robot_models){
        base::samples::Joints joint_state = randomJointState(it.second->jointLimits());
        joint_state.time = base::Time::now();
        it.second->update(joint_state);
    }

    map<string,QPSolverPtr> solvers;
    solvers["qpoases"] = make_shared<QPOASESSolver>();
    solvers["eiquadprog"] = make_shared<EiquadprogSolver>();
    solvers["qpswift"] = make_shared<QPSwiftSolver>();

    const string root = "RH5_Root_Link", tip = "LLAnkle_FT";
    evaluateVelocitySceneQuadraticCost(robot_models, solvers, root, tip, n_samples, robot);
    evaluateAccelerationSceneTSID(robot_models, solvers, root, tip, n_samples, robot);
}

void runRH5LegsBenchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Legs Model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
    cfg.floating_base = true;
    cfg.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    cfg.contact_points.elements = {wbc::ActiveContact(1,0.6),wbc::ActiveContact(1,0.6)};
    const string robot = "rh5_legs";

    map<string,RobotModelPtr> robot_models;
    robot_models["kdl"] =  make_shared<RobotModelKDL>();
    robot_models["hyrodyn"] =  make_shared<RobotModelHyrodyn>();
    robot_models["pinocchio"] =  make_shared<RobotModelPinocchio>();
    robot_models["rbdl"] =  make_shared<RobotModelRBDL>();
    for(auto it : robot_models){
        if(!it.second->configure(cfg)) abort();
    }
    robot_models["hyrodyn_hybrid"] =  make_shared<RobotModelHyrodyn>();
    cfg.file = "../../../models/rh5/urdf/rh5_legs_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs_hybrid.yml";
    if(!robot_models["hyrodyn_hybrid"]->configure(cfg)) abort();
    for(auto it : robot_models){
        base::samples::Joints joint_state = randomJointState(it.second->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state;
        floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
        floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
        floating_base_state.twist.setZero();
        floating_base_state.acceleration.setZero();
        floating_base_state.time = base::Time::now();
        joint_state.time = base::Time::now();
        it.second->update(joint_state, floating_base_state);
    }

    map<string,QPSolverPtr> solvers;
    solvers["qpoases"] = make_shared<QPOASESSolver>();
    solvers["eiquadprog"] = make_shared<EiquadprogSolver>();
    solvers["qpswift"] = make_shared<QPSwiftSolver>();

    const string root = "world", tip = "LLAnkle_FT";
    evaluateVelocitySceneQuadraticCost(robot_models, solvers, root, tip, n_samples, robot);
    evaluateAccelerationSceneTSID(robot_models, solvers, root, tip, n_samples, robot);
}

void runRH5Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5 Model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5/urdf/rh5.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
    cfg.floating_base = true;
    cfg.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    cfg.contact_points.elements = {wbc::ActiveContact(1,0.6),wbc::ActiveContact(1,0.6)};
    const string robot = "rh5";

    map<string,RobotModelPtr> robot_models;
    robot_models["kdl"] =  make_shared<RobotModelKDL>();
    robot_models["hyrodyn"] =  make_shared<RobotModelHyrodyn>();
    robot_models["pinocchio"] =  make_shared<RobotModelPinocchio>();
    robot_models["rbdl"] =  make_shared<RobotModelRBDL>();
    for(auto it : robot_models){
        if(!it.second->configure(cfg)) abort();
    }
    robot_models["hyrodyn_hybrid"] =  make_shared<RobotModelHyrodyn>();
    cfg.file = "../../../models/rh5/urdf/rh5_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5/hyrodyn/rh5_hybrid.yml";
    if(!robot_models["hyrodyn_hybrid"]->configure(cfg)) abort();
    for(auto it : robot_models){
        base::samples::Joints joint_state = randomJointState(it.second->jointLimits());
        base::samples::RigidBodyStateSE3 floating_base_state;
        floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
        floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
        floating_base_state.twist.setZero();
        floating_base_state.acceleration.setZero();
        floating_base_state.time = base::Time::now();
        joint_state.time = base::Time::now();
        it.second->update(joint_state, floating_base_state);
    }


    map<string,QPSolverPtr> solvers;
    solvers["qpoases"] = make_shared<QPOASESSolver>();
    solvers["eiquadprog"] = make_shared<EiquadprogSolver>();
    solvers["qpswift"] = make_shared<QPSwiftSolver>();

    const string root = "world", tip = "LLAnkle_FT";
    //evaluateVelocitySceneQuadraticCost(robot_models, solvers, root, tip, n_samples, robot);
    evaluateAccelerationSceneTSID(robot_models, solvers, root, tip, n_samples, robot);
}

void runRH5v2Benchmarks(int n_samples){
    cout << " ----------- Evaluating RH5v2 model -----------" << endl;
    RobotModelConfig cfg;
    cfg.file = "../../../models/rh5v2/urdf/rh5v2.urdf";
    cfg.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
    const string robot = "rh5v2";

    map<string,RobotModelPtr> robot_models;
    robot_models["kdl"] =  make_shared<RobotModelKDL>();
    robot_models["hyrodyn"] =  make_shared<RobotModelHyrodyn>();
    robot_models["pinocchio"] =  make_shared<RobotModelPinocchio>();
    robot_models["rbdl"] =  make_shared<RobotModelRBDL>();
    for(auto it : robot_models){
        if(!it.second->configure(cfg)) abort();
    }
    robot_models["hyrodyn_hybrid"] =  make_shared<RobotModelHyrodyn>();
    cfg.file = "../../../models/rh5v2/urdf/rh5v2_hybrid.urdf";
    cfg.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    if(!robot_models["hyrodyn_hybrid"]->configure(cfg)) abort();
    for(auto it : robot_models){
        base::samples::Joints joint_state = randomJointState(it.second->jointLimits());
        base::samples::RigidBodyStateSE3 rbs = randomFloatingBaseState();
        joint_state.time = base::Time::now();
        it.second->update(joint_state, rbs);
    }

    map<string,QPSolverPtr> solvers;
    solvers["qpoases"] = make_shared<QPOASESSolver>();
    solvers["eiquadprog"] = make_shared<EiquadprogSolver>();
    solvers["qpswift"] = make_shared<QPSwiftSolver>();

    const string root = "RH5v2_Root_Link", tip = "ALWristFT_Link";
    evaluateVelocitySceneQuadraticCost(robot_models, solvers, root, tip, n_samples, robot);
    evaluateAccelerationSceneTSID(robot_models, solvers, root, tip, n_samples, robot);
}

void runBenchmarks(int n_samples){
    boost::filesystem::create_directory("results");

    runRH5LegsBenchmarks(n_samples);
    runRH5Benchmarks(n_samples);
    runKUKAIiwaBenchmarks(n_samples);
    runRH5SingleLegBenchmarks(n_samples);
    runRH5v2Benchmarks(n_samples);
}

int
main(){

    srand(time(NULL));
    int n_samples = 10;
    runBenchmarks(n_samples);
}
