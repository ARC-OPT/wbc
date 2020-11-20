#include "robot_models/RobotModelHyrodyn.hpp"
#include "robot_models/RobotModelKDL.hpp"
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

BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn){

    const string base_link = "RH5_Root_Link";
    const string ee_link = "LLAnkle_FT";
    vector<RobotModelConfig> configs;
    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF));
    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(configs) == true);
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
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state));

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(base_link, ee_link);

    cout<<"KDL Model: Computing Joint Space inertia matrix ..."<<endl;
    base::Time start = base::Time::now();
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::Time end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"KDL Model: Computing Bias forces..."<<endl;
    start = base::Time::now();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    RobotModelHyrodyn robot_model_hyrodyn;
    configs.clear();
    configs.push_back(RobotModelConfig("../../../models/hyrodyn/rh5/rh5_submechanisms.yml",
                                       {},{},
                                       wbc::ModelType::SUBMECHANISM));
    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF));
    robot_model_hyrodyn.configure(configs);
    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state));

    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_kdl.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, ee_link);

    cout<<"Hyrodyn Model: Computing Joint Space inertia matrix ..."<<endl;
    start = base::Time::now();
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"Hyrodyn Model: Computing Bias forces..."<<endl;
    start = base::Time::now();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"Robot Model KDL"<<endl;
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
    cout<<C_hyrodyn.transpose()<<endl<<endl;

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
}


BOOST_AUTO_TEST_CASE(compare_kdl_vs_hyrodyn_floating_base){
    const string base_link = "world";
    const string ee_link = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    vector<RobotModelConfig> configs;

    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z",
                                        "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF,
                                       true,
                                       "world",
                                       floating_base_state));
    RobotModelKDL robot_model_kdl;
    BOOST_CHECK(robot_model_kdl.configure(configs) == true);
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

    cout<<"KDL Model: Computing Joint Space inertia matrix ..."<<endl;
    base::Time start = base::Time::now();
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::Time end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"KDL Model: Computing Bias forces..."<<endl;
    start = base::Time::now();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    RobotModelHyrodyn robot_model_hyrodyn;
    configs.clear();
    configs.push_back(RobotModelConfig("../../../models/hyrodyn/rh5/rh5_submechanisms_floating_base.yml",
                                       {},{},
                                       wbc::ModelType::SUBMECHANISM));
    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                        "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z",
                                        "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF,
                                       true,
                                       "world",
                                       floating_base_state));

    robot_model_hyrodyn.configure(configs);
    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state, floating_base_state));


    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_hyrodyn.rigidBodyState(base_link, ee_link);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, ee_link);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, ee_link);

    cout<<"Hyrodyn Model: Computing Joint Space inertia matrix ..."<<endl;
    start = base::Time::now();
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"Hyrodyn Model: Computing Bias forces..."<<endl;
    start = base::Time::now();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    end = base::Time::now();
    cout<<"... took "<<(end-start).toSeconds()<<" seconds"<<endl;

    cout<<"Robot Model KDL"<<endl;
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
    cout<<C_hyrodyn.transpose()<<endl<<endl;

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
}
