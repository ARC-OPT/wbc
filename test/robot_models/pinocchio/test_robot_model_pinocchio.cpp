#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
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

BOOST_AUTO_TEST_CASE(compare_kdl_vs_pinocchio){

    srand(time(NULL));

    string root = "RH5_Root_Link", tip = "ARWrist_FT";
    string urdf_file = "../../../../models/rh5/urdf/rh5.urdf";
    double default_accuracy = 1e-4;

    RobotModelConfig config;
    RobotModelPinocchio robot_model_pinocchio;
    config = RobotModelConfig(urdf_file);
    BOOST_CHECK(robot_model_pinocchio.configure(config) == true);
    base::samples::Joints joint_state;
    joint_state.names = robot_model_pinocchio.jointNames();
    joint_state.elements.resize(robot_model_pinocchio.noOfJoints());
    for(int i = 0; i < robot_model_pinocchio.noOfJoints(); i++){
        joint_state[i].position = (double)rand()/RAND_MAX;
        joint_state[i].speed = (double)rand()/RAND_MAX;
        joint_state[i].acceleration = (double)rand()/RAND_MAX;
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model_pinocchio.update(joint_state));
    base::samples::RigidBodyStateSE3 rbs_pinocchio, com_pinocchio;
    base::VectorXd bias_forces_pinocchio;
    base::MatrixXd space_jac_pinocchio, body_jac_pinocchio, com_jac_pinocchio, jnt_space_inertia_mat_pinocchio;
    base::Acceleration spatial_acc_bias_pinocchio;
    BOOST_CHECK_NO_THROW(rbs_pinocchio = robot_model_pinocchio.rigidBodyState(root, tip));
    BOOST_CHECK_NO_THROW(space_jac_pinocchio = robot_model_pinocchio.spaceJacobian(root, tip));
    BOOST_CHECK_NO_THROW(body_jac_pinocchio = robot_model_pinocchio.bodyJacobian(root, tip));
    BOOST_CHECK_NO_THROW(com_jac_pinocchio = robot_model_pinocchio.comJacobian());
    BOOST_CHECK_NO_THROW(jnt_space_inertia_mat_pinocchio = robot_model_pinocchio.jointSpaceInertiaMatrix());
    BOOST_CHECK_NO_THROW(bias_forces_pinocchio = robot_model_pinocchio.biasForces());
    BOOST_CHECK_NO_THROW(spatial_acc_bias_pinocchio = robot_model_pinocchio.spatialAccelerationBias(root, tip));
    BOOST_CHECK_NO_THROW(com_pinocchio = robot_model_pinocchio.centerOfMass());

    // Compare to KDL based model
    RobotModelHyrodyn robot_model_kdl;
    config.submechanism_file = "../../../../models/rh5/hyrodyn/rh5.yml";
    BOOST_CHECK(robot_model_kdl.configure(config) == true);
    BOOST_CHECK_NO_THROW(robot_model_kdl.update(joint_state));

    base::samples::RigidBodyStateSE3 rbs_kdl, com_kdl;
    base::MatrixXd space_jac_kdl, body_jac_kdl, com_jac_kdl, jnt_space_inertia_mat_kdl;
    base::VectorXd bias_forces_kdl;
    base::Acceleration spatial_acc_bias_kdl;
    BOOST_CHECK_NO_THROW(rbs_kdl = robot_model_kdl.rigidBodyState(root, tip));
    BOOST_CHECK_NO_THROW(space_jac_kdl = robot_model_kdl.spaceJacobian(root, tip));
    BOOST_CHECK_NO_THROW(body_jac_kdl = robot_model_kdl.bodyJacobian(root, tip));
    BOOST_CHECK_NO_THROW(com_jac_kdl = robot_model_kdl.comJacobian());
    BOOST_CHECK_NO_THROW(jnt_space_inertia_mat_kdl = robot_model_kdl.jointSpaceInertiaMatrix());
    BOOST_CHECK_NO_THROW(bias_forces_kdl = robot_model_kdl.biasForces());
    BOOST_CHECK_NO_THROW(spatial_acc_bias_kdl = robot_model_kdl.spatialAccelerationBias(root, tip));
    BOOST_CHECK_NO_THROW(com_kdl = robot_model_kdl.centerOfMass());

    cout<<"--------- Robot Model Pinocchio ---------"<<endl;
    /*cout<<"Position        "<<rbs_pinocchio.pose.position.transpose()<<endl;
    cout<<"Orientation     "<<rbs_pinocchio.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist (linear)  "<<rbs_pinocchio.twist.linear.transpose()<<endl;
    cout<<"Twist (angular) "<<rbs_pinocchio.twist.angular.transpose()<<endl;
    cout<<"Acc (linear)    "<<rbs_pinocchio.acceleration.linear.transpose()<<endl;
    cout<<"Acc (angular)   "<<rbs_pinocchio.acceleration.angular.transpose()<<endl<<endl;
    cout<<"Space Jacobian: "<<endl;
    cout<<space_jac_pinocchio<<endl;
    cout<<"Body Jacobian: "<<endl;
    cout<<body_jac_pinocchio<<endl;
    cout<<"CoM Jacobian: "<<endl;
    cout<<com_jac_pinocchio<<endl;
    cout<<"Joint Space inertia matrix: "<<endl;
    cout<<jnt_space_inertia_mat_pinocchio<<endl;
    cout<<"Bias forces: "<<endl;
    cout<<bias_forces_pinocchio.transpose()<<endl;
    cout<<"CoM position:     "<<com_pinocchio.pose.position.transpose()<<endl;
    cout<<"CoM velocity:     "<<com_pinocchio.twist.linear.transpose()<<endl;
    cout<<"CoM acceleration: "<<com_pinocchio.acceleration.linear.transpose()<<endl;

    cout<<"------------ Robot Model KDL ------------"<<endl;
    cout<<"Position        "<<rbs_kdl.pose.position.transpose()<<endl;
    cout<<"Orientation     "<<rbs_kdl.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist (linear)  "<<rbs_kdl.twist.linear.transpose()<<endl;
    cout<<"Twist (angular) "<<rbs_kdl.twist.angular.transpose()<<endl;
    cout<<"Acc (linear)    "<<rbs_kdl.acceleration.linear.transpose()<<endl;
    cout<<"Acc (angular)   "<<rbs_kdl.acceleration.angular.transpose()<<endl<<endl;
    cout<<"Space Jacobian: "<<endl;
    cout<<space_jac_kdl<<endl;
    cout<<"Body Jacobian: "<<endl;
    cout<<body_jac_kdl<<endl;
    cout<<"CoM Jacobian: "<<endl;
    cout<<com_jac_kdl<<endl;
    cout<<"Joint Space inertia matrix: "<<endl;
    cout<<jnt_space_inertia_mat_kdl<<endl;
    cout<<"Bias forces: "<<endl;
    cout<<bias_forces_kdl.transpose()<<endl;
    cout<<"CoM position:     "<<com_kdl.pose.position.transpose()<<endl;
    cout<<"CoM velocity:     "<<com_kdl.twist.linear.transpose()<<endl;
    cout<<"CoM acceleration: "<<com_kdl.acceleration.linear.transpose()<<endl;*/

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(rbs_kdl.pose.position[i] - rbs_pinocchio.pose.position[i]) < default_accuracy);
        BOOST_CHECK(fabs(rbs_kdl.pose.orientation.coeffs()[i] - rbs_pinocchio.pose.orientation.coeffs()[i]) < default_accuracy);
        BOOST_CHECK(fabs(rbs_kdl.twist.linear[i] - rbs_pinocchio.twist.linear[i]) < default_accuracy);
        BOOST_CHECK(fabs(rbs_kdl.twist.angular[i] - rbs_pinocchio.twist.angular[i]) < default_accuracy);
        BOOST_CHECK(fabs(rbs_kdl.acceleration.linear[i] - rbs_pinocchio.acceleration.linear[i]) < default_accuracy);
        BOOST_CHECK(fabs(rbs_kdl.acceleration.angular[i] - rbs_pinocchio.acceleration.angular[i]) < default_accuracy);
    }
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < robot_model_pinocchio.noOfJoints(); j++){
            string joint_name = robot_model_pinocchio.jointNames()[j];
            int joint_idx = robot_model_kdl.jointIndex(joint_name);
            BOOST_CHECK(fabs(space_jac_kdl(i,joint_idx) - space_jac_pinocchio(i,j)) < default_accuracy);
            BOOST_CHECK(fabs(body_jac_kdl(i,joint_idx) - body_jac_pinocchio(i,j)) < default_accuracy);
        }
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < robot_model_pinocchio.noOfJoints(); j++){
            string joint_name = robot_model_pinocchio.jointNames()[j];
            int joint_idx = robot_model_kdl.jointIndex(joint_name);
            BOOST_CHECK(fabs(com_jac_kdl(i,joint_idx) - com_jac_pinocchio(i,j)) < default_accuracy);
        }
    }
    for(int i = 0; i < robot_model_pinocchio.noOfJoints(); i++){
        for(int j = 0; j < robot_model_pinocchio.noOfJoints(); j++){
            int row_idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[i]);
            int col_idx = robot_model_kdl.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(jnt_space_inertia_mat_kdl(row_idx,col_idx) - jnt_space_inertia_mat_pinocchio(i,j)) < default_accuracy);
        }
    }
    for(int i = 0; i < robot_model_pinocchio.noOfJoints(); i++){
        string joint_name = robot_model_pinocchio.jointNames()[i];
        int joint_idx = robot_model_kdl.jointIndex(joint_name);
        BOOST_CHECK(fabs(bias_forces_kdl(joint_idx) - bias_forces_pinocchio(i)) < default_accuracy);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(spatial_acc_bias_kdl.linear[i] - spatial_acc_bias_pinocchio.linear[i]) < default_accuracy);
        BOOST_CHECK(fabs(spatial_acc_bias_kdl.angular[i] - spatial_acc_bias_pinocchio.angular[i]) < default_accuracy);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_pinocchio.pose.position[i] - com_kdl.pose.position[i]) < default_accuracy);
        BOOST_CHECK(fabs(com_pinocchio.twist.linear[i] - com_kdl.twist.linear[i]) < default_accuracy);
        //BOOST_CHECK(fabs(com_pinocchio.acceleration.linear[i] - com_kdl.acceleration.linear[i]) < default_accuracy);
    }

}
