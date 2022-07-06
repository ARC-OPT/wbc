#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include "tools/URDFTools.hpp"
#include <random>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configuration){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelPinocchio robot_model;

    vector<string> joint_names = {"kuka_lbr_l_joint_1",
                                            "kuka_lbr_l_joint_2",
                                            "kuka_lbr_l_joint_3",
                                            "kuka_lbr_l_joint_4",
                                            "kuka_lbr_l_joint_5",
                                            "kuka_lbr_l_joint_6",
                                            "kuka_lbr_l_joint_7"};
    vector<string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                          "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};

    // Valid config
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.independentJointNames()[i]);
        BOOST_CHECK(i == robot_model.jointIndex(joint_names[i]));
    }
}

void compareRobotModels(RobotModelPinocchio robot_model_pinocchio, RobotModelKDL robot_model_kdl, const string frame_id ){

    int nj = robot_model_pinocchio.noOfJoints();

    base::VectorXd q(nj),qd(nj),qdd(nj);
    for(int i = 0; i < nj; i++){
        q(i) = (double)rand()/RAND_MAX;
        qd(i) = (double)rand()/RAND_MAX;
        qdd(i) = (double)rand()/RAND_MAX;
    }

    base::samples::Joints joint_state;
    joint_state.resize(nj);
    joint_state.names = robot_model_kdl.actuatedJointNames();
    for(size_t i = 0; i < nj; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = qd[i];
        joint_state[i].acceleration = qdd[i];
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state));
    BOOST_CHECK_NO_THROW(robot_model_pinocchio.update(joint_state));

    string base_link = robot_model_pinocchio.worldFrame();

    base::samples::RigidBodyStateSE3 rbs_kdl = robot_model_kdl.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_kdl = robot_model_kdl.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_kdl = robot_model_kdl.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_kdl = robot_model_kdl.jointSpaceInertiaMatrix();
    base::MatrixXd C_kdl = robot_model_kdl.biasForces();
    base::Acceleration acc_kdl  = robot_model_kdl.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_kdl = robot_model_kdl.centerOfMass();
    base::MatrixXd com_jac_kdl = robot_model_kdl.comJacobian();

    base::samples::RigidBodyStateSE3 rbs_pinocchio = robot_model_pinocchio.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_pinocchio = robot_model_pinocchio.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_pinocchio = robot_model_pinocchio.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_pinocchio = robot_model_pinocchio.jointSpaceInertiaMatrix();
    base::MatrixXd C_pinocchio = robot_model_pinocchio.biasForces();
    base::Acceleration acc_pinocchio  = robot_model_pinocchio.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_pinocchio = robot_model_pinocchio.centerOfMass();
    base::MatrixXd com_jac_pinocchio = robot_model_pinocchio.comJacobian();

    cout<<"................... Robot Model KDL ..........................."<<endl;
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
    cout<<"Linear: "<<acc_pinocchio.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_pinocchio.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_kdl<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_kdl.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<"Position:     "<<com_kdl.pose.position.transpose()<<endl;
    cout<<"Velocity:     "<<com_kdl.twist.linear.transpose()<<endl;
    cout<<"Acceleration: "<<com_kdl.acceleration.linear.transpose()<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_kdl<<endl<<endl;

    cout<<".....................Robot Model Pinocchio ......................."<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs_pinocchio.pose.position.transpose()<<endl;
    cout<<rbs_pinocchio.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs_pinocchio.twist.linear.transpose()<<endl;
    cout<<rbs_pinocchio.twist.angular.transpose()<<endl;
    cout<<"Space Jacobian"<<endl;
    cout<<Js_pinocchio<<endl;
    cout<<"Body Jacobian"<<endl;
    cout<<Jb_pinocchio<<endl;
    cout<<"Spatial Acc Bias"<<endl;
    cout<<"Linear: "<<acc_kdl.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_kdl.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_pinocchio<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_pinocchio.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<"Position:     "<<com_pinocchio.pose.position.transpose()<<endl;
    cout<<"Velocity:     "<<com_pinocchio.twist.linear.transpose()<<endl;
    cout<<"Acceleration: "<<com_pinocchio.acceleration.linear.transpose()<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_pinocchio<<endl<<endl;

    cout<<".........................................................."<<endl;

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.position(i) - rbs_pinocchio.pose.position(i)) < 1e-3);
    if(rbs_pinocchio.pose.orientation.w() < 0 && rbs_kdl.pose.orientation.w() > 0 ||
       rbs_pinocchio.pose.orientation.w() > 0 && rbs_kdl.pose.orientation.w() < 0)
        rbs_kdl.pose.orientation = base::Quaterniond(-rbs_kdl.pose.orientation.w(),-rbs_kdl.pose.orientation.x(),-rbs_kdl.pose.orientation.y(),-rbs_kdl.pose.orientation.z());
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs_kdl.pose.orientation.coeffs()(i) - rbs_pinocchio.pose.orientation.coeffs()(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.linear(i) - rbs_pinocchio.twist.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.twist.angular(i) - rbs_pinocchio.twist.angular(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.acceleration.linear(i) - rbs_pinocchio.acceleration.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_kdl.acceleration.angular(i) - rbs_pinocchio.acceleration.angular(i)) < 1e-3);
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(Js_kdl(i,idx) - Js_pinocchio(i,j)) < 1e-3);
        }
    }
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(Jb_kdl(i,idx) - Jb_pinocchio(i,j)) < 1e-3);
        }
    }
    for(int i = 0; i < nj; i++){
        for(int j = 0; j < nj; j++){
            int row_idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[i]);
            int col_idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(H_kdl(row_idx,col_idx) - H_pinocchio(i,j)) < 1e-2);
        }
    }
    for(int i = 0; i < nj; i++){
        uint idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[i]);
        BOOST_CHECK(fabs(C_kdl(idx) - C_pinocchio(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_kdl.linear(i) - acc_pinocchio.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(acc_kdl.angular(i) - acc_pinocchio.angular(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_kdl.pose.position(i) - com_pinocchio.pose.position(i)) < 1e-3);
        //BOOST_CHECK(fabs(com_kdl.twist.linear(i) - com_pinocchio.twist.linear(i)) < 1e-3);
        //BOOST_CHECK(fabs(com_kdl.acceleration.linear(i) - com_pinocchio.acceleration.linear(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(com_jac_kdl(i,idx) - com_jac_pinocchio(i,j)) < 1e-3);
        }
    }
}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_pinocchio){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Pinocchio-based robot model
     */

    srand(time(NULL));

    const string frame_id = "LLAnkle_FT";

    RobotModelPinocchio robot_model_pinocchio;
    RobotModelConfig config("../../../../models/rh5/urdf/rh5.urdf");
    BOOST_CHECK(robot_model_pinocchio.configure(config) == true);

    RobotModelKDL robot_model_kdl;
    config = RobotModelConfig("../../../../models/rh5/urdf/rh5.urdf");
    BOOST_CHECK(robot_model_kdl.configure(config) == true);

    compareRobotModels(robot_model_pinocchio, robot_model_kdl, frame_id);
}
