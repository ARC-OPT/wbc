#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include <boost/test/unit_test.hpp>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configuration){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelHyrodyn robot_model;

    std::vector<std::string> joint_names = {"kuka_lbr_l_joint_1",
                                            "kuka_lbr_l_joint_2",
                                            "kuka_lbr_l_joint_3",
                                            "kuka_lbr_l_joint_4",
                                            "kuka_lbr_l_joint_5",
                                            "kuka_lbr_l_joint_6",
                                            "kuka_lbr_l_joint_7"};
    std::vector<std::string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                                    "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};

    // Valid config
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    BOOST_CHECK(robot_model.configure(config) == true);

    // Invalid filename
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urd");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Invalid submechanism file
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.ym";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with floating base
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    config.floating_base_state.pose.fromTransform(Eigen::Affine3d::Identity());
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid floating base state
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    config.floating_base_state.pose.position.setZero();
    config.floating_base_state.pose.orientation = base::Vector4d(1,1,1,1);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with blacklisted joints
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_blacklist.yml";
    config.joint_blacklist.push_back(joint_names[6]);
    config.floating_base = false;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid joints in blacklist
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_blacklist.yml";
    config.joint_blacklist.push_back("kuka_lbr_l_joint_X");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with contact points
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    config.contact_points.names.push_back("kuka_lbr_l_tcp");
    config.contact_points.elements.push_back(1);
    config.joint_blacklist.clear();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid contact points
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    config.contact_points.names.push_back("XYZ");
    config.contact_points.elements.push_back(1);
    BOOST_CHECK(robot_model.configure(config) == false);
}

void compareRobotModels(RobotModelHyrodyn robot_model_hyrodyn, RobotModelKDL robot_model_kdl, const string frame_id){

    int nj = robot_model_hyrodyn.noOfActuatedJoints();

    base::VectorXd q(nj),qd(nj),qdd(nj);
    for(int i = 0; i < nj; i++){
        q(i) = (double)rand()/RAND_MAX;
        qd(i) = (double)rand()/RAND_MAX;
        qdd(i) = (double)rand()/RAND_MAX;
    }

    base::samples::Joints joint_state;
    joint_state.resize(nj);
    joint_state.names = robot_model_hyrodyn.actuatedJointNames();
    for(size_t i = 0; i < nj; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = qd[i];
        joint_state[i].acceleration = qdd[i];
    }
    joint_state.time = base::Time::now();

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = joint_state.time;

    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state, floating_base_state));
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state, floating_base_state));

    string base_link = robot_model_hyrodyn.worldFrame();

    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_hyrodyn.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    base::Acceleration acc_hyrodyn  = robot_model_hyrodyn.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_hyrodyn = robot_model_hyrodyn.centerOfMass();
    base::MatrixXd com_jac_hyrodyn = robot_model_hyrodyn.comJacobian();

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    base::Acceleration acc_kdl  = robot_model_kdl.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_kdl = robot_model_kdl.centerOfMass();
    base::MatrixXd com_jac_kdl = robot_model_kdl.comJacobian();

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.position(i) - rbs_hyrodyn.pose.position(i)) < 1e-3);
    if(rbs_hyrodyn.pose.orientation.w() < 0 && rbs_kdl.pose.orientation.w() > 0 ||
       rbs_hyrodyn.pose.orientation.w() > 0 && rbs_kdl.pose.orientation.w() < 0)
        rbs_kdl.pose.orientation = base::Quaterniond(-rbs_kdl.pose.orientation.w(),-rbs_kdl.pose.orientation.x(),-rbs_kdl.pose.orientation.y(),-rbs_kdl.pose.orientation.z());
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.orientation.coeffs()(i) - rbs_hyrodyn.pose.orientation.coeffs()(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.linear(i) - rbs_hyrodyn.twist.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.angular(i) - rbs_hyrodyn.twist.angular(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.acceleration.linear(i) - rbs_hyrodyn.acceleration.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.acceleration.angular(i) - rbs_hyrodyn.acceleration.angular(i)) < 1e-3);
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[j]);
            BOOST_CHECK(fabs(Js_kdl(i,idx) - Js_hyrodyn(i,j)) < 1e-3);
        }
    }
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[j]);
            BOOST_CHECK(fabs(Jb_kdl(i,idx) - Jb_hyrodyn(i,j)) < 1e-3);
        }
    }
    for(int i = 0; i < nj; i++){
        for(int j = 0; j < nj; j++){
            int row_idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[i]);
            int col_idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[j]);
            BOOST_CHECK(fabs(H_kdl(row_idx,col_idx) - H_hyrodyn(i,j)) < 1e-2);
        }
    }
    for(int i = 0; i < nj; i++){
        uint idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[i]);
        BOOST_CHECK(fabs(C_kdl(idx) - C_hyrodyn(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_kdl.linear(i) - acc_hyrodyn.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular(i) - acc_hyrodyn.angular(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_kdl.pose.position(i) - com_hyrodyn.pose.position(i)) < 1e-3);
        //BOOST_CHECK(fabs(com_kdl.twist.linear(i) - com_hyrodyn.twist.linear(i)) < 1e-3);
        //BOOST_CHECK(fabs(com_kdl.acceleration.linear(i) - com_hyrodyn.acceleration.linear(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_hyrodyn.jointNames()[j]);
            BOOST_CHECK(fabs(com_jac_kdl(i,idx) - com_jac_hyrodyn(i,j)) < 1e-3);
        }
    }

    cout<<"................... Robot Model Hyrodyn ..........................."<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs_hyrodyn.pose.position.transpose()<<endl;
    cout<<rbs_hyrodyn.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs_hyrodyn.twist.linear.transpose()<<endl;
    cout<<rbs_hyrodyn.twist.angular.transpose()<<endl;
    cout<<"Space Jacobian"<<endl;
    cout<<Js_hyrodyn<<endl;
    cout<<"Body Jacobian"<<endl;
    cout<<Jb_hyrodyn<<endl;
    cout<<"Spatial Acc Bias"<<endl;
    cout<<"Linear: "<<acc_hyrodyn.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_hyrodyn.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_hyrodyn<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_hyrodyn.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<com_hyrodyn.pose.position.transpose()<<endl<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_hyrodyn<<endl<<endl;
    cout<<".....................Robot Model KDL ......................."<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs_kdl.pose.position.transpose()<<endl;
    cout<<rbs_kdl.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs_kdl.twist.linear.transpose()<<endl;
    cout<<rbs_kdl.twist.angular.transpose()<<endl;
    cout<<"Space Jacobian"<<endl;
    cout<<Js_kdl<<endl;
    cout<<"Body Jacobian"<<endl;
    cout<<Jb_kdl<<endl;
    cout<<"Spatial Acc Bias"<<endl;
    cout<<"Linear: "<<acc_kdl.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_kdl.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_kdl<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_kdl.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<com_kdl.pose.position.transpose()<<endl<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_kdl<<endl<<endl;
    cout<<".........................................................."<<endl;
}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Hyrodyn-based robot model
     */

    srand(time(NULL));

    const string frame_id = "kuka_lbr_l_tcp";

    RobotModelHyrodyn robot_model_hyrodyn;
    RobotModelConfig config("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    BOOST_CHECK(robot_model_hyrodyn.configure(config) == true);

    RobotModelKDL robot_model_kdl;
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    BOOST_CHECK(robot_model_kdl.configure(config) == true);

    compareRobotModels(robot_model_hyrodyn, robot_model_kdl, frame_id);
}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn_floating_base){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Hyrodyn-based robot model including floating base
     */

    srand(time(NULL));
    const string frame_id = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config("../../../../models/rh5/urdf/rh5.urdf");
    config.floating_base = true;
    config.floating_base_state = floating_base_state;

    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(config) == true);
    uint na = robot_model_kdl.noOfActuatedJoints();

    RobotModelHyrodyn robot_model_hyrodyn;
    config.submechanism_file = "../../../../models/rh5/hyrodyn/rh5.yml";
    robot_model_hyrodyn.configure(config);

    compareRobotModels(robot_model_hyrodyn, robot_model_kdl, frame_id);
}

BOOST_AUTO_TEST_CASE(compare_serial_vs_hybrid_model){

    /**
     * Check if the differential inverse kinematics solution of a serial robot and the equivalent a series-parallel hybrid robot model match
     */

    string root = "RH5_Root_Link";
    string tip  = "LLAnklePitch_Link";

    RobotModelHyrodyn robot_model_hybrid;
    RobotModelConfig config_hybrid("../../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf",
                                   {"LLHip1", "LLHip2",
                                    "LLHip3", "LLHip3_B11", "LLHip3_Act1",
                                    "LLKnee", "LLKnee_B11", "LLKnee_Act1",
                                    "LLAnkleRoll", "LLAnklePitch", "LLAnkle_E11", "LLAnkle_E21", "LLAnkle_B11", "LLAnkle_B12", "LLAnkle_Act1", "LLAnkle_B21", "LLAnkle_B22", "LLAnkle_Act2"},
                                   {"LLHip1", "LLHip2", "LLHip3_Act1","LLKnee_Act1", "LLAnkle_Act1", "LLAnkle_Act2"});
    config_hybrid.submechanism_file = "../../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    BOOST_CHECK(robot_model_hybrid.configure(config_hybrid) == true);


    RobotModelHyrodyn robot_model_serial;
    RobotModelConfig config_serial("../../../../models/rh5/urdf/rh5_single_leg.urdf",
                                   {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                   {"LLHip1", "LLHip2", "LLHip3","LLKnee", "LLAnkleRoll", "LLAnklePitch"});
    config_serial.submechanism_file = "../../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    BOOST_CHECK(robot_model_serial.configure(config_serial) == true);

    base::samples::Joints joint_state;
    joint_state.names = robot_model_hybrid.hyrodynHandle()->jointnames_independent;
    for(auto n : robot_model_hybrid.hyrodynHandle()->jointnames_independent){
        base::JointState js;
        js.position = js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    joint_state["LLKnee"].position = 1.5;
    joint_state["LLAnklePitch"].position = -0.7;

    robot_model_serial.update(joint_state);
    robot_model_hybrid.update(joint_state);

    cout<<"******************** HYBRID MODEL *****************"<<endl;
    base::MatrixXd jac = robot_model_hybrid.spaceJacobian(root, tip);
    base::Vector6d v;
    v.setZero();
    v[2] = -0.1;
    base::VectorXd u = jac.completeOrthogonalDecomposition().pseudoInverse()*v;
    robot_model_hybrid.hyrodynHandle()->ud = u;
    robot_model_hybrid.hyrodynHandle()->calculate_forward_system_state();

    cout<< "Solution actuation space" << endl;
    std::cout<<robot_model_hybrid.hyrodynHandle()->ud.transpose()<<endl;

    cout<< "Solution projected to independent joint space" << endl;
    std::cout<<robot_model_hybrid.hyrodynHandle()->yd.transpose()<<endl;

    cout<<"******************** SERIAL MODEL *****************"<<endl;
    jac = robot_model_serial.spaceJacobian(root, tip);
    base::VectorXd yd = jac.completeOrthogonalDecomposition().pseudoInverse()*v;

    cout<< "Solution independent joint space" << endl;
    std::cout<<yd.transpose()<<endl;

    for(int i = 0; i < robot_model_hybrid.noOfActuatedJoints(); i++)
        BOOST_CHECK(fabs(robot_model_hybrid.hyrodynHandle()->yd[i] - yd[i]) < 1e-6);
}

/*BOOST_AUTO_TEST_CASE(floating_base_test)
{
    srand(time(NULL));

    string urdf_filename = "../../../../models/kuka/urdf/kuka_iiwa.urdf";

    wbc::RobotModelHyrodyn robot_model;

    base::samples::RigidBodyStateSE3 floating_base_state;
    base::Vector3d euler(0s,0,0);
    floating_base_state.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.twist.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.time = base::Time::now();

    RobotModelConfig config(urdf_filename);
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base_state = floating_base_state;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);

    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfActuatedJoints());
    joint_state.names = robot_model.actuatedJointNames();
    for(int i = 0; i < robot_model.noOfActuatedJoints(); i++){
        joint_state[i].position = double(rand())/RAND_MAX;
        joint_state[i].speed = double(rand())/RAND_MAX;
        joint_state[i].acceleration = double(rand())/RAND_MAX;
    }

    joint_state.time = base::Time::now();
    robot_model.update(joint_state, floating_base_state);

    // (1) Check if the floating base state is correct in general, i.e., compare with the configured state

    base::samples::RigidBodyStateSE3 floating_base_state_measured = robot_model.rigidBodyState("world", "kuka_lbr_l_link_0");

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(floating_base_state_measured.pose.position(i) - floating_base_state.pose.position(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.twist.linear(i) - floating_base_state.twist.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.twist.angular(i) - floating_base_state.twist.angular(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.acceleration.linear(i) - floating_base_state.acceleration.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.acceleration.angular(i) - floating_base_state.acceleration.angular(i)) < 1e-3);
    }
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(floating_base_state_measured.pose.orientation.coeffs()(i) - floating_base_state.pose.orientation.coeffs()(i)) < 1e-3);

    cout<<"Expected floating base state"<<endl;
    cout<<"Position:               "<<floating_base_state.pose.position.transpose()<<endl;
    cout<<"Orientation:            "<<floating_base_state.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist (linear):         "<<floating_base_state.twist.linear.transpose()<<endl;
    cout<<"Twist (angular):        "<<floating_base_state.twist.angular.transpose()<<endl;
    cout<<"Acceleration (linear):  "<<floating_base_state.acceleration.linear.transpose()<<endl;
    cout<<"Acceleration (angular): "<<floating_base_state.acceleration.angular.transpose()<<endl<<endl;

    cout<<"Measured floating base state"<<endl;
    cout<<"Position:               "<<floating_base_state_measured.pose.position.transpose()<<endl;
    cout<<"Orientation:            "<<floating_base_state_measured.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist (linear):         "<<floating_base_state_measured.twist.linear.transpose()<<endl;
    cout<<"Twist (angular):        "<<floating_base_state_measured.twist.angular.transpose()<<endl;
    cout<<"Acceleration (linear):  "<<floating_base_state_measured.acceleration.linear.transpose()<<endl;
    cout<<"Acceleration (angular): "<<floating_base_state_measured.acceleration.angular.transpose()<<endl<<endl;
}*/
