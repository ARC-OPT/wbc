#include <robot_models/RobotModelKDL.hpp>
#include <tools/URDFTools.hpp>
#include <iostream>

using namespace wbc;
using namespace std;

int main(){

    RobotModelConfig model_config;
    model_config.file = "../../../examples/teleoperation/urdf/recupera_exo_and_rh5.urdf";
    model_config.joint_names = URDFTools::jointNamesFromURDF(model_config.file);
    model_config.actuated_joint_names = model_config.joint_names;

    // Create overall model
    RobotModelKDL robot_model;
    if(!robot_model.configure(model_config))
        return -1;

    // update state of all joints in model, set all positions to zero right now
    base::samples::Joints joint_state;
    for(auto n : robot_model.jointNames()){
        base::JointState st;
        st.position = 0;
        st.speed = 0;
        st.acceleration = 0;
        joint_state.names.push_back(n);
        joint_state.elements.push_back(st);
    }
    joint_state.time = base::Time::now();
    robot_model.update(joint_state);

    ///// Example computations

    // Compute pose and twist of left_exo_hand_ft of exo system expressed in ALWrist_FT of RH5 system
    base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState("ALWrist_FT", "left_exo_hand_ft");

    cout<<"ALWrist_FT -> left_exo_hand_ft"<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs.pose.position.transpose()<<endl;
    cout<<rbs.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs.twist.linear.transpose()<<endl;
    cout<<rbs.twist.angular.transpose()<<endl;

    // To transform poses you can use the toTransform() method

    // For poses
    base::Pose pose_left_exo_hand_ft_in_ALWrist_FT = rbs.pose;
    base::Pose pose_ALWrist_FT_in_root = robot_model.rigidBodyState("root_link_upper_body_exo", "ALWrist_FT").pose;
    base::Pose pose_left_exo_hand_ft_in_root;
    pose_left_exo_hand_ft_in_root.fromTransform(pose_ALWrist_FT_in_root.toTransform()*pose_left_exo_hand_ft_in_ALWrist_FT.toTransform());

    cout<<"Pose left_exo_hand_ft in ALWrist_FT"<<endl;
    cout<<pose_left_exo_hand_ft_in_ALWrist_FT.position.transpose()<<endl;
    cout<<pose_left_exo_hand_ft_in_ALWrist_FT.orientation.coeffs().transpose()<<endl;
    cout<<"Pose left_exo_hand_ft in root"<<endl;
    cout<<pose_left_exo_hand_ft_in_root.position.transpose()<<endl;
    cout<<pose_left_exo_hand_ft_in_root.orientation.coeffs().transpose()<<endl;

    // To transform twist and wrenches you can use the *-operator implemented in base::RigidBodyStateSE3

    // For twists
    base::Twist twist_in_ALWrist_FT(base::Vector3d(1,0,0),base::Vector3d(0,0,0));
    base::Twist twist_in_left_exo_hand_ft = rbs.pose*twist_in_ALWrist_FT;

    cout<<"Twist in ALWrist_FT"<<endl;
    cout<<twist_in_ALWrist_FT.linear.transpose()<<endl;
    cout<<twist_in_ALWrist_FT.angular.transpose()<<endl;
    cout<<"Twist transformed to left_exo_hand_ft"<<endl;
    cout<<twist_in_left_exo_hand_ft.linear.transpose()<<endl;
    cout<<twist_in_left_exo_hand_ft.angular.transpose()<<endl;

    // For wrenches
    base::Wrench wrench_in_ALWrist_FT(base::Vector3d(1,0,0),base::Vector3d(0,0,0));
    base::Wrench wrench_in_left_exo_hand_ft = rbs.pose*wrench_in_ALWrist_FT;

    cout<<"Twist in ALWrist_FT"<<endl;
    cout<<wrench_in_ALWrist_FT.force.transpose()<<endl;
    cout<<wrench_in_ALWrist_FT.torque.transpose()<<endl;
    cout<<"Twist transformed to left_exo_hand_ft"<<endl;
    cout<<wrench_in_left_exo_hand_ft.force.transpose()<<endl;
    cout<<wrench_in_left_exo_hand_ft.torque.transpose()<<endl;

    return 0;
}
