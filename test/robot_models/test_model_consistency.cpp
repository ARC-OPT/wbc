#include <boost/test/unit_test.hpp>
#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
#include "robot_models/rbdl/RobotModelRBDL.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "test_robot_model.hpp"

using namespace wbc;
using namespace std;

void compareRbs(base::samples::RigidBodyStateSE3 rbs_a, base::samples::RigidBodyStateSE3 rbs_b, double default_accuracy = 1e-3){
    if(rbs_a.hasValidPose() && rbs_b.hasValidPose()){
        for(int i = 0; i < 3; i++) BOOST_CHECK(fabs(rbs_a.pose.position(i) - rbs_b.pose.position(i)) < default_accuracy);
        if((rbs_b.pose.orientation.w() < 0 && rbs_a.pose.orientation.w() > 0) ||
           (rbs_b.pose.orientation.w() > 0 && rbs_a.pose.orientation.w() < 0))
            rbs_a.pose.orientation = base::Quaterniond(-rbs_a.pose.orientation.w(),-rbs_a.pose.orientation.x(),-rbs_a.pose.orientation.y(),-rbs_a.pose.orientation.z());
        for(int i = 0; i < 4; i++) BOOST_CHECK(fabs(rbs_a.pose.orientation.coeffs()(i) - rbs_b.pose.orientation.coeffs()(i)) < default_accuracy);
    }
    if(rbs_a.hasValidTwist() && rbs_b.hasValidTwist()){
        for(int i = 0; i < 3; i++) BOOST_CHECK(fabs(rbs_a.twist.linear(i) - rbs_b.twist.linear(i)) < default_accuracy);
        for(int i = 0; i < 3; i++) BOOST_CHECK(fabs(rbs_a.twist.angular(i) - rbs_b.twist.angular(i)) < default_accuracy);
    }
    if(rbs_a.hasValidAcceleration() && rbs_b.hasValidAcceleration()){
        for(int i = 0; i < 3; i++) BOOST_CHECK(fabs(rbs_a.acceleration.linear(i) - rbs_b.acceleration.linear(i)) < default_accuracy);
        for(int i = 0; i < 3; i++) BOOST_CHECK(fabs(rbs_a.acceleration.angular(i) - rbs_b.acceleration.angular(i)) < default_accuracy);
    }
}

uint jointIdx(vector<string> joint_names, string joint_name){
    uint idx = std::find(joint_names.begin(), joint_names.end(), joint_name) - joint_names.begin();
    if(idx >= joint_names.size())
        throw std::invalid_argument("Invalid joint name");
    return idx;
}

void compareJacobian(base::MatrixXd mat_a, base::MatrixXd mat_b, vector<string> joint_names_a, vector<string> joint_names_b, bool floating_base, double default_accuracy = 1e-3){
    assert(mat_a.rows() == mat_b.rows());
    assert(mat_a.cols() == mat_b.cols());
    uint start_idx = 0;
    if(floating_base) // Floating base is modeled differently in each model. Thus, the first 6 columns of Jacobian never match
        start_idx = 6;
    for(int i = 0; i < mat_a.rows(); i++){
        for(int j = start_idx; j < mat_a.cols(); j++){
            uint idx_col = jointIdx(joint_names_a, joint_names_b[j]);
            BOOST_CHECK(fabs(mat_a(i,idx_col) - mat_b(i,j)) < default_accuracy);
        }
    }
}

void compareMassInertiaMat(base::MatrixXd mat_a, base::MatrixXd mat_b, vector<string> joint_names_a, vector<string> joint_names_b, double default_accuracy = 1e-3){
    assert(mat_a.rows() == mat_b.rows());
    assert(mat_a.cols() == mat_b.cols());
    for(int i = 0; i < mat_a.rows(); i++){
        uint idx_row = jointIdx(joint_names_a, joint_names_b[i]);
        for(int j = 0; j < mat_a.cols(); j++){
            uint idx_col = jointIdx(joint_names_a, joint_names_b[j]);
            BOOST_CHECK(fabs(mat_a(idx_row,idx_col) - mat_b(i,j)) < default_accuracy);
        }
    }
}

void compareVect(base::VectorXd vect_a, base::VectorXd vect_b, vector<string> joint_names_a, vector<string> joint_names_b, double default_accuracy = 1e-3){
    assert(vect_a.size() == vect_b.size());
    for(int i = 0; i < vect_a.size(); i++){
        uint idx = jointIdx(joint_names_a, joint_names_b[i]);
        BOOST_CHECK(fabs(vect_a(idx) - vect_b(i)) < default_accuracy);
    }
}

void compareRobotModels(RobotModelConfig cfg, string tip_frame, bool verbose = false){

    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(cfg));

    RobotModelPinocchio robot_model_pinocchio;
    BOOST_CHECK(robot_model_pinocchio.configure(cfg));

    RobotModelRBDL robot_model_rbdl;
    BOOST_CHECK(robot_model_rbdl.configure(cfg));

    RobotModelHyrodyn robot_model_hyrodyn;
    BOOST_CHECK(robot_model_hyrodyn.configure(cfg));

    if(verbose){
        cout << "---------------- Joint order ----------------" << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        for(auto n : robot_model_kdl.jointNames()) cout<<n<<" "; cout<<endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        for(auto n : robot_model_pinocchio.jointNames()) cout<<n<<" "; cout<<endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        for(auto n : robot_model_rbdl.jointNames()) cout<<n<<" "; cout<<endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        for(auto n : robot_model_hyrodyn.jointNames()) cout<<n<<" "; cout<<endl;
    }

    base::samples::Joints joint_state = makeRandomJointState(robot_model_kdl.actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state = makeRandomFloatingBaseState();

    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state,floating_base_state));
    BOOST_CHECK_NO_THROW(robot_model_pinocchio.update(joint_state,floating_base_state));
    BOOST_CHECK_NO_THROW(robot_model_rbdl.update(joint_state,floating_base_state));
    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state,floating_base_state));

    // FK

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(robot_model_kdl.worldFrame(), tip_frame);
    base::samples::RigidBodyStateSE3 rbs_pinocchio = robot_model_pinocchio.rigidBodyState(robot_model_pinocchio.worldFrame(), tip_frame);
    base::samples::RigidBodyStateSE3 rbs_rbdl = robot_model_rbdl.rigidBodyState(robot_model_rbdl.worldFrame(), tip_frame);
    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_hyrodyn.rigidBodyState(robot_model_hyrodyn.worldFrame(), tip_frame);

    compareRbs(rbs_kdl, rbs_pinocchio);
    compareRbs(rbs_kdl, rbs_rbdl);
    compareRbs(rbs_kdl, rbs_hyrodyn);

    if(verbose){
        cout << "---------------- Forward Kinematics ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        printRbs(rbs_kdl);
        cout<< " .......... RobotModelPinocchio .........." << endl;
        printRbs(rbs_pinocchio);
        cout<< " .......... RobotModelRBDL .........." << endl;
        printRbs(rbs_rbdl);
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        printRbs(rbs_hyrodyn);
    }

    // Space Jacobian

    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(robot_model_kdl.worldFrame(), tip_frame);
    base::MatrixXd Js_pinocchio = robot_model_pinocchio.spaceJacobian(robot_model_pinocchio.worldFrame(), tip_frame);
    base::MatrixXd Js_rbdl = robot_model_rbdl.spaceJacobian(robot_model_rbdl.worldFrame(), tip_frame);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(robot_model_hyrodyn.worldFrame(), tip_frame);

    compareJacobian(Js_kdl,Js_pinocchio,robot_model_kdl.jointNames(), robot_model_pinocchio.jointNames(), cfg.floating_base);
    compareJacobian(Js_kdl,Js_rbdl,robot_model_kdl.jointNames(), robot_model_rbdl.jointNames(), cfg.floating_base);
    compareJacobian(Js_kdl,Js_hyrodyn,robot_model_kdl.jointNames(), robot_model_hyrodyn.jointNames(), cfg.floating_base);

    if(verbose){
        cout << "---------------- Space Jacobian ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        cout << Js_kdl << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout << Js_pinocchio << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout << Js_rbdl << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout << Js_hyrodyn << endl << endl;
    }

    // Body Jacobian

    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(robot_model_kdl.worldFrame(), tip_frame);
    base::MatrixXd Jb_pinocchio = robot_model_pinocchio.bodyJacobian(robot_model_pinocchio.worldFrame(), tip_frame);
    base::MatrixXd Jb_rbdl = robot_model_rbdl.bodyJacobian(robot_model_rbdl.worldFrame(), tip_frame);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(robot_model_hyrodyn.worldFrame(), tip_frame);

    compareJacobian(Jb_kdl,Jb_pinocchio,robot_model_kdl.jointNames(), robot_model_pinocchio.jointNames(), cfg.floating_base);
    compareJacobian(Jb_kdl,Jb_rbdl,robot_model_kdl.jointNames(), robot_model_rbdl.jointNames(), cfg.floating_base);
    compareJacobian(Jb_kdl,Jb_hyrodyn,robot_model_kdl.jointNames(), robot_model_hyrodyn.jointNames(), cfg.floating_base);

    if(verbose){
        cout << "---------------- Body Jacobian ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl ;
        cout << Jb_kdl << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout << Jb_pinocchio << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout << Jb_rbdl << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout << Jb_hyrodyn << endl << endl;
    }

    // CoM Jacobian

    base::MatrixXd Jcom_kdl = robot_model_kdl.comJacobian();
    base::MatrixXd Jcom_pinocchio = robot_model_pinocchio.comJacobian();
    base::MatrixXd Jcom_rbdl = robot_model_rbdl.comJacobian();
    base::MatrixXd Jcom_hyrodyn = robot_model_hyrodyn.comJacobian();

    compareJacobian(Jcom_kdl,Jcom_pinocchio,robot_model_kdl.jointNames(), robot_model_pinocchio.jointNames(), cfg.floating_base);
    compareJacobian(Jcom_kdl,Jcom_rbdl,robot_model_kdl.jointNames(), robot_model_rbdl.jointNames(), cfg.floating_base);
    compareJacobian(Jcom_kdl,Jcom_hyrodyn,robot_model_kdl.jointNames(), robot_model_hyrodyn.jointNames(), cfg.floating_base);

    if(verbose){
        cout << "---------------- CoM Jacobian ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        cout << Jcom_kdl << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout << Jcom_pinocchio << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout << Jcom_rbdl << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout << Jcom_hyrodyn << endl << endl;
    }

    // Center of Mass

    base::samples::RigidBodyStateSE3 com_kdl = robot_model_kdl.centerOfMass();
    base::samples::RigidBodyStateSE3 com_pinocchio = robot_model_pinocchio.centerOfMass();
    base::samples::RigidBodyStateSE3 com_rbdl = robot_model_rbdl.centerOfMass();
    base::samples::RigidBodyStateSE3 com_hyrodyn = robot_model_hyrodyn.centerOfMass();

    compareRbs(com_kdl,com_pinocchio);
    compareRbs(com_kdl,com_rbdl);
    compareRbs(com_kdl,com_hyrodyn);

    if(verbose){
        cout << "---------------- CoM ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        printRbs(com_kdl);
        cout<< " .......... RobotModelPinocchio .........." << endl;
        printRbs(com_pinocchio);
        cout<< " .......... RobotModelRBDL .........." << endl;
        printRbs(com_rbdl);
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        printRbs(com_hyrodyn);
    }

    // Spatial acceleration bias

    base::Acceleration acc_kdl = robot_model_kdl.spatialAccelerationBias(robot_model_kdl.worldFrame(), tip_frame);
    base::Acceleration acc_pinocchio = robot_model_pinocchio.spatialAccelerationBias(robot_model_pinocchio.worldFrame(), tip_frame);
    base::Acceleration acc_rbdl = robot_model_rbdl.spatialAccelerationBias(robot_model_rbdl.worldFrame(), tip_frame);
    base::Acceleration acc_hyrodyn = robot_model_hyrodyn.spatialAccelerationBias(robot_model_hyrodyn.worldFrame(), tip_frame);

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_kdl.linear[i] - acc_pinocchio.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.linear[i] - acc_rbdl.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.linear[i] - acc_hyrodyn.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular[i] - acc_pinocchio.angular[i]) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular[i] - acc_rbdl.angular[i]) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular[i] - acc_hyrodyn.angular[i]) < 1e-3);
    }

    if(verbose){
        cout << "---------------- Spatial Acceleration Bias ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        cout<< acc_kdl.linear.transpose() << " " << acc_kdl.angular.transpose() << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout<< acc_pinocchio.linear.transpose() << " " << acc_pinocchio.angular.transpose() << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout<< acc_rbdl.linear.transpose() << " " << acc_rbdl.angular.transpose() << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout<< acc_hyrodyn.linear.transpose() << " " << acc_hyrodyn.angular.transpose() << endl << endl;
    }

    // Bias Forces

    base::VectorXd C_kdl = robot_model_kdl.biasForces();
    base::VectorXd C_pinocchio = robot_model_pinocchio.biasForces();
    base::VectorXd C_rbdl = robot_model_rbdl.biasForces();
    base::VectorXd C_hyrodyn = robot_model_hyrodyn.biasForces();

    compareVect(C_kdl,C_pinocchio,robot_model_kdl.jointNames(), robot_model_pinocchio.jointNames());
    compareVect(C_kdl,C_rbdl,robot_model_kdl.jointNames(), robot_model_rbdl.jointNames());
    compareVect(C_kdl,C_hyrodyn,robot_model_kdl.jointNames(), robot_model_hyrodyn.jointNames());

    if(verbose){
        cout << "---------------- Bias Forces ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        cout<< C_kdl.transpose() << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout<< C_pinocchio.transpose() << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout<< C_rbdl.transpose() << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout<< C_hyrodyn.transpose() << endl << endl;
    }

    // Mass-inertia matrix

    base::MatrixXd Hq_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::MatrixXd Hq_pinocchio = robot_model_pinocchio.jointSpaceInertiaMatrix();
    base::MatrixXd Hq_rbdl = robot_model_rbdl.jointSpaceInertiaMatrix();
    base::MatrixXd Hq_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();

    compareMassInertiaMat(Hq_kdl,Hq_pinocchio,robot_model_kdl.jointNames(), robot_model_pinocchio.jointNames());
    compareMassInertiaMat(Hq_kdl,Hq_rbdl,robot_model_kdl.jointNames(), robot_model_rbdl.jointNames());
    compareMassInertiaMat(Hq_kdl,Hq_hyrodyn,robot_model_kdl.jointNames(), robot_model_hyrodyn.jointNames());

    if(verbose){
        cout << "---------------- Mass-Inertia Matrix ----------------" << endl << endl;
        cout<< " .......... RobotModelKDL .........." << endl;
        cout<< Hq_kdl << endl << endl;
        cout<< " .......... RobotModelPinocchio .........." << endl;
        cout<< Hq_pinocchio << endl << endl;
        cout<< " .......... RobotModelRBDL .........." << endl;
        cout<< Hq_rbdl << endl << endl;
        cout<< " .......... RobotModelHyrodyn .........." << endl;
        cout<< Hq_hyrodyn << endl << endl;
    }

}

BOOST_AUTO_TEST_CASE(fixed_base){

    vector<string> urdf_files = {"../../../models/rh5/urdf/rh5_single_leg.urdf",
                                 "../../../models/kuka/urdf/kuka_iiwa.urdf",
                                 "../../../models/rh5v2/urdf/rh5v2.urdf"};
    vector<string> sub_mec_files = {"../../../models/rh5/hyrodyn/rh5_single_leg.yml",
                                   "../../../models/kuka/hyrodyn/kuka_iiwa.yml",
                                   "../../../models/rh5v2/hyrodyn/rh5v2.yml"};
    vector<string> tip_frames = {"LLAnkle_FT", "kuka_lbr_l_tcp", "ALWristFT_Link"};
    bool verbose = false;

    for(uint i = 0; i < urdf_files.size(); i++){

        if(verbose){
            cout<<"------------------- Robot Model --------------------"<<endl;
            cout<<urdf_files[i]<<endl<<endl;
        }

        RobotModelConfig cfg(urdf_files[i]);
        cfg.submechanism_file = sub_mec_files[i];
        compareRobotModels(cfg, tip_frames[i], verbose);
    }
}

BOOST_AUTO_TEST_CASE(floating_base){

    string urdf_file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    string sub_mec_file = "../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    string tip_frame = "kuka_lbr_l_tcp";
    bool verbose = false;

    RobotModelConfig cfg(urdf_file);
    cfg.submechanism_file = sub_mec_file;
    cfg.floating_base = true;

    compareRobotModels(cfg, tip_frame, verbose);
}
