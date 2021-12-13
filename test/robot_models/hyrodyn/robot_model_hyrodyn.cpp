#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include <boost/test/unit_test.hpp>

using namespace std;
using namespace wbc;

double whiteNoise(const double std_dev)
{
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

BOOST_AUTO_TEST_CASE(configuration_test){

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
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    BOOST_CHECK(robot_model.configure(config) == true);

    // Invalid filename
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urd";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Invalid submechanism file
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.ym";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with floating base
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid floating base state
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    config.floating_base_state.pose.orientation = base::Vector4d(1,1,1,1);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with blacklisted joints
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_blacklist.yml";
    config.joint_blacklist.push_back(joint_names[6]);
    config.floating_base = false;
    BOOST_CHECK(robot_model.configure(config) == true);


    // Config with invalid joints in blacklist
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_blacklist.yml";
    config.joint_blacklist.push_back("kuka_lbr_l_joint_X");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with contact points
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    config.contact_points.push_back("kuka_lbr_l_tcp");
    config.joint_blacklist.clear();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid contact points
    config.file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    config.contact_points.push_back("XYZ");
    BOOST_CHECK(robot_model.configure(config) == false);

}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Hyrodyn-based robot model
     */

    const string base_link = "RH5_Root_Link";
    const string ee_link = "LLAnkle_FT";
    RobotModelConfig config("../../../../models/rh5/urdf/rh5_one_leg.urdf",
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"});
    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(config) == true);
    uint na = robot_model_kdl.noOfActuatedJoints();

    base::VectorXd q(na),qd(na),qdd(na);
    q << 0,0,-0.2,0.4,-0.2,0;
    qd.setZero();
    qdd.setZero();
    for(int i = 0; i < na; i++){
        q(i) += whiteNoise(1e-4);
        qd(i) += 0.5 + whiteNoise(1e-4);
        qdd(i) += 0.2 + whiteNoise(1e-4);
    }

    base::samples::Joints joint_state;
    joint_state.resize(na);
    joint_state.names = robot_model_kdl.actuatedJointNames();
    for(size_t i = 0; i < na; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = qd[i];
        joint_state[i].acceleration = qdd[i];
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state));

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(base_link, ee_link);
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    base::Acceleration acc_kdl  = robot_model_kdl.spatialAccelerationBias(base_link,ee_link);

    RobotModelHyrodyn robot_model_hyrodyn;
    config = RobotModelConfig("../../../../models/rh5/urdf/rh5_one_leg.urdf",
                             {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                             {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"});
    config.submechanism_file = "../../../../models/rh5/hyrodyn/rh5_one_leg.yml";
    robot_model_hyrodyn.configure(config);
    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state));

    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_kdl.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, ee_link);
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    base::Acceleration acc_hyrodyn  = robot_model_hyrodyn.spatialAccelerationBias(base_link,ee_link);

    /*cout<<"Robot Model KDL"<<endl;
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
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_kdl<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_kdl.transpose()<<endl<<endl;

    cout<<"Robot Model Hyrodyn"<<endl;
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
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_hyrodyn<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_hyrodyn.transpose()<<endl<<endl;*/

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.position(i) - rbs_hyrodyn.pose.position(i)) < 1e-9);
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.orientation.coeffs()(i) - rbs_hyrodyn.pose.orientation.coeffs()(i)) < 1e-9);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.linear(i) - rbs_hyrodyn.twist.linear(i)) < 1e-9);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.angular(i) - rbs_hyrodyn.twist.angular(i)) < 1e-9);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(Js_kdl(i,j) - Js_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(Jb_kdl(i,j) - Jb_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < na; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(H_kdl(i,j) - H_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < na; i++)
        BOOST_CHECK(fabs(C_kdl(i) - C_hyrodyn(i)) < 1e-3);
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_kdl.linear(i) - acc_hyrodyn.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular(i) - acc_hyrodyn.angular(i)) < 1e-3);
    }
}


BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn_floating_base){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Hyrodyn-based robot model when using a floating base
     */

    const string base_link = "world";
    const string ee_link = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config("../../../../models/rh5/urdf/rh5_one_leg.urdf",
                           {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z",
                            "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                            true,
                            "world",
                            floating_base_state,
                            std::vector<std::string>(),
                            "../../../../models/rh5/hyrodyn/rh5_one_leg_floating_base.yml");
    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(config) == true);
    uint na = robot_model_kdl.noOfActuatedJoints();

    base::VectorXd q(na),qd(na),qdd(na);
    q << 0,0,-0.2,0.4,-0.2,0;
    qd.setZero();
    qdd.setZero();
    for(int i = 0; i < na; i++){
        q(i) += whiteNoise(1e-4);
        qd(i) += whiteNoise(1e-4);
        qdd(i) += whiteNoise(1e-4);
    }

    base::samples::Joints joint_state;
    joint_state.resize(na);
    joint_state.names = robot_model_kdl.actuatedJointNames();
    for(size_t i = 0; i < na; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = qd[i];
        joint_state[i].acceleration = qdd[i];
    }
    joint_state.time = base::Time::now();
    floating_base_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state, floating_base_state));

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(base_link, ee_link);
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    base::Acceleration acc_kdl  = robot_model_kdl.spatialAccelerationBias(base_link,ee_link);

    RobotModelHyrodyn robot_model_hyrodyn;
    robot_model_hyrodyn.configure(config);
    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state, floating_base_state));


    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_hyrodyn.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, ee_link);
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    base::Acceleration acc_hyrodyn  = robot_model_hyrodyn.spatialAccelerationBias(base_link,ee_link);

    /*cout<<"Robot Model KDL"<<endl;
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
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_kdl<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_kdl.transpose()<<endl<<endl;

    cout<<"Robot Model Hyrodyn"<<endl;
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
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_hyrodyn<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_hyrodyn.transpose()<<endl<<endl;*/

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.position(i) - rbs_hyrodyn.pose.position(i)) < 1e-6);
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.orientation.coeffs()(i) - rbs_hyrodyn.pose.orientation.coeffs()(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.linear(i) - rbs_hyrodyn.twist.linear(i)) < 1e-9);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.angular(i) - rbs_hyrodyn.twist.angular(i)) < 1e-9);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(Js_kdl(i,j) - Js_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(Jb_kdl(i,j) - Jb_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < na; i++)
        for(int j = 0; j < na; j++)
            BOOST_CHECK(fabs(H_kdl(i,j) - H_hyrodyn(i,j)) < 1e-3);
    for(int i = 0; i < na; i++)
        BOOST_CHECK(fabs(C_kdl(i) - C_hyrodyn(i)) < 1e-3);
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_kdl.linear(i) - acc_hyrodyn.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular(i) - acc_hyrodyn.angular(i)) < 1e-3);
    }
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
    if(!robot_model_hybrid.configure(config_hybrid))
        abort();

    RobotModelHyrodyn robot_model_serial;
    RobotModelConfig config_serial("../../../../models/rh5/urdf/rh5_one_leg.urdf",
                                   {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                   {"LLHip1", "LLHip2", "LLHip3","LLKnee", "LLAnkleRoll", "LLAnklePitch"});
    config_serial.submechanism_file = "../../../../models/rh5/hyrodyn/rh5_one_leg.yml";
    if(!robot_model_serial.configure(config_serial))
        abort();

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
    std::cout<<robot_model_hybrid.hyrodynHandle()->yd.transpose()<<endl;

    cout<< "Solution projected to independent joint space" << endl;
    std::cout<<robot_model_hybrid.hyrodynHandle()->yd.transpose()<<endl;


    cout<<"******************** SERIAL MODEL *****************"<<endl;
    jac = robot_model_serial.spaceJacobian(root, tip);
    base::VectorXd y = jac.completeOrthogonalDecomposition().pseudoInverse()*v;

    cout<< "Solution independent joint space" << endl;
    std::cout<<y.transpose()<<endl;
}
