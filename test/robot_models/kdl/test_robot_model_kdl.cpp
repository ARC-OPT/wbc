#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/kdl/KinematicChainKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "tools/URDFTools.hpp"
#include <regex>

using namespace std;
using namespace wbc;

string rootDir(){
    std::string root_dir = string(__FILE__);
    const size_t last_slash_idx = root_dir.rfind('/');
    root_dir =  root_dir.substr(0, last_slash_idx) + "/../../..";
    return root_dir;
}

BOOST_AUTO_TEST_CASE(configuration_test){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelKDL robot_model;

    std::vector<std::string> joint_names = {"kuka_lbr_l_joint_1",
                                            "kuka_lbr_l_joint_2",
                                            "kuka_lbr_l_joint_3",
                                            "kuka_lbr_l_joint_4",
                                            "kuka_lbr_l_joint_5",
                                            "kuka_lbr_l_joint_6",
                                            "kuka_lbr_l_joint_7"};
    std::vector<std::string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                                    "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};

    std::string root_dir = rootDir();

    // Valid config
    config.file = root_dir + "/models/kuka/urdf/kuka_iiwa.urdf";
    BOOST_CHECK(robot_model.configure(config) == true);

    // Invalid filename
    config.file = root_dir + "/models/kuka/urdf/kuka_iiwa.urd";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Empty filename
    config.file = "";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with joint names
    config.file = root_dir + "/models/kuka/urdf/kuka_iiwa.urdf";
    config.joint_names = joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Valid config with joint names and actuated joint names
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Valid config with actuated joint names only
    config.actuated_joint_names = joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Missing joint name
    config.joint_names = {"kuka_lbr_l_joint_1",
                          "kuka_lbr_l_joint_2",
                          "kuka_lbr_l_joint_3",
                          "kuka_lbr_l_joint_4",
                          "kuka_lbr_l_joint_5",
                          "kuka_lbr_l_joint_6"};
    config.actuated_joint_names = config.joint_names;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Invalid joint name
    config.joint_names = {"kuka_lbr_l_joint_1",
                          "kuka_lbr_l_joint_2",
                          "kuka_lbr_l_joint_3",
                          "kuka_lbr_l_joint_4",
                          "kuka_lbr_l_joint_5",
                          "kuka_lbr_l_joint_6",
                          "kuka_lbr_l_joint_X"};
    config.actuated_joint_names = config.joint_names;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Less actuated joint names than joints
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    config.actuated_joint_names.pop_back();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Invalid actuated joint name
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    config.actuated_joint_names[6] = "kuka_lbr_l_joint_X";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with floating base
    config.joint_names = floating_base_names + joint_names;
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid floating base name
    config.joint_names = floating_base_names + joint_names;
    config.joint_names[0] = "floating_base_trans_";
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with missing floating base name
    config.joint_names = floating_base_names + joint_names;
    config.joint_names.erase(config.joint_names.begin());
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with invalid floating base state
    config.joint_names = floating_base_names + joint_names;
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    config.floating_base_state.pose.orientation = base::Vector4d(1,1,1,1);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with blacklisted joints
    config.joint_names = joint_names;
    config.joint_names.pop_back();
    config.actuated_joint_names = config.joint_names;
    config.joint_blacklist.push_back(joint_names[6]);
    config.floating_base = false;
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with blacklisted joints and missing joint name
    config.joint_names = joint_names;
    config.joint_names.pop_back();
    config.joint_names.pop_back();
    config.actuated_joint_names = config.joint_names;
    config.joint_blacklist.push_back(joint_names[6]);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with invalid joints in blacklist
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.joint_blacklist.push_back("kuka_lbr_l_joint_X");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with contact points
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.contact_points.push_back("kuka_lbr_l_tcp");
    config.joint_blacklist.clear();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid contact points
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.contact_points.push_back("XYZ");
    BOOST_CHECK(robot_model.configure(config) == false);

}

BOOST_AUTO_TEST_CASE(verify_jacobian_and_forward_kinematics){

    /**
     * Verify WBC Forward Kinematics and Jacobian for a single joint system
     */

    vector<string> joint_names;
    joint_names.push_back("base_to_rot");
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    string urdf_model_file = rootDir() + "/models/others/urdf/single_joint.urdf";

    RobotModelKDL robot_model;
    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_model_file, joint_names, joint_names)) == true);

    double t = 0;
    base::VectorXd joint_vel(joint_state.size());
    base::VectorXd joint_acc(joint_state.size());
    while(t < 1){
        joint_state[joint_state.mapNameToIndex("base_to_rot")].position     = sin(t);
        joint_state[joint_state.mapNameToIndex("base_to_rot")].speed        = cos(t);
        joint_state[joint_state.mapNameToIndex("base_to_rot")].acceleration = -sin(t);
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_vel(i) = joint_state[i].speed;
            joint_acc(i) = joint_state[i].acceleration;
        }
        joint_state.time = base::Time::now();
        robot_model.update(joint_state);

        base::samples::RigidBodyStateSE3 cstate = robot_model.rigidBodyState("base", "ee");
        base::Vector3d euler = base::getEuler(cstate.pose.orientation);

        double zero = 0.0;
        double pos = joint_state[joint_state.mapNameToIndex("base_to_rot")].position;
        double vel = joint_state[joint_state.mapNameToIndex("base_to_rot")].speed;
        double acc = joint_state[joint_state.mapNameToIndex("base_to_rot")].acceleration;

        /*printf("Expected position: %.4f %.4f %.4f\n",   zero, -sin(pos), cos(pos));
        printf("Computed Position: %.4f %.4f %.4f\n\n",   cstate.pose.position(0), cstate.pose.position(1), cstate.pose.position(2));*/
        BOOST_CHECK(fabs(zero - cstate.pose.position(0)) <= 1e-7);
        BOOST_CHECK(fabs(-sin(pos) - cstate.pose.position(1)) <= 1e-7);
        BOOST_CHECK(fabs(cos(pos) - cstate.pose.position(2)) <= 1e-7);

        /*printf("Expected Orientation: %.4f %.4f %.4f\n",   zero, zero, pos);
        printf("Computed Orientation: %.4f %.4f %.4f\n\n",   euler(0),  euler(1), euler(2));*/
        BOOST_CHECK(fabs(zero -  euler(0)) <= 1e-7);
        BOOST_CHECK(fabs(zero - euler(1)) <= 1e-7);
        BOOST_CHECK(fabs(pos - euler(2)) <= 1e-7);

        /*printf("Expected Linear Vel:  %.4f %.4f %.4f\n",   zero, -vel*cos(pos), -vel*sin(pos));
        printf("Computed Linear Vel:  %.4f %.4f %.4f\n\n",   cstate.twist.linear(0), cstate.twist.linear(1), cstate.twist.linear(2));*/
        BOOST_CHECK(fabs(zero - cstate.twist.linear(0)) <= 1e-7);
        BOOST_CHECK(fabs(-vel*cos(pos) - cstate.twist.linear(1)) <= 1e-7);
        BOOST_CHECK(fabs(-vel*sin(pos) - cstate.twist.linear(2)) <= 1e-7);

        /*printf("Expected Angular Vel:  %.4f %.4f %.4f\n",   vel, zero, zero);
        printf("Computed Angular Vel:  %.4f %.4f %.4f\n\n",   cstate.twist.angular(0), cstate.twist.angular(1), cstate.twist.angular(2));*/
        BOOST_CHECK(fabs(vel - cstate.twist.angular(0)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.twist.angular(1)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.twist.angular(2)) <= 1e-7);

        base::VectorXd twist = robot_model.spaceJacobian("base", "ee") * joint_vel;
        /*printf("Linear Vel from Jacobian: %.4f %.4f %.4f\n", twist(0), twist(1), twist(2));
        printf("Angular Vel from Jacobian: %.4f %.4f %.4f\n\n", twist(3), twist(4), twist(5));*/
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(twist(i) - cstate.twist.linear(i)) <= 1e-7);
            BOOST_CHECK(fabs(twist(i+3) - cstate.twist.angular(i)) <= 1e-7);
        }

        base::VectorXd acceleration = robot_model.jacobianDot("base", "ee")*joint_vel + robot_model.spaceJacobian("base", "ee")*joint_acc;
        double expected_y_acc = -acc*cos(pos) +  vel*vel*sin(pos);
        double expected_z_acc = -acc*sin(pos) -vel*vel*cos(pos);
        /*printf("Expected Linear Acc:  %.4f %.4f %.4f\n",   zero, expected_y_acc, expected_z_acc);
        printf("Computed Linear Acc:  %.4f %.4f %.4f\n\n",   acceleration(0), acceleration(1), acceleration(2));*/
        BOOST_CHECK(fabs(zero - acceleration(0)) <= 1e-7);
        BOOST_CHECK(fabs(expected_y_acc - acceleration(1)) <= 1e-7);
        BOOST_CHECK(fabs(expected_z_acc - acceleration(2)) <= 1e-7);

        /*printf("Expected Angular Acc:  %.4f %.4f %.4f\n",   acc, zero, zero);
        printf("Computed Angular Acc:  %.4f %.4f %.4f\n\n",   acceleration(3), acceleration(4), acceleration(5));*/
        BOOST_CHECK(fabs(acc - acceleration(3)) <= 1e-7);
        BOOST_CHECK(fabs(zero - acceleration(4)) <= 1e-7);
        BOOST_CHECK(fabs(zero - acceleration(5)) <= 1e-7);


        //printf("...........................................................\n");
        usleep(0.1*1000*1000);
        t+=0.1;
    }
}

BOOST_AUTO_TEST_CASE(compare_forward_kinematics_wbc_vs_kdl){

    /**
     * Compare forward kinematics for KDL-based robot model in WBC with pure KDL solution
     */

    string urdf_model_file = rootDir() + "/models/kuka/urdf/kuka_iiwa.urdf";
    string root = "kuka_lbr_l_link_0";
    string tip  = "kuka_lbr_l_tcp";

    base::samples::Joints joint_state;
    vector<string> joint_names = URDFTools::jointNamesFromURDF(urdf_model_file);
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    RobotModelKDL robot_model;
    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_model_file, joint_names, joint_names, false)) == true);

    KDL::Chain chain;
    BOOST_CHECK_NO_THROW(robot_model.getTree().getChain(root, tip, chain));

    KDL::ChainFkSolverVel_recursive vel_solver(chain);
    KDL::JntArrayVel q_and_q_dot(joint_names.size());

    double t = 0;
    while(t < 1){
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_1")].position     = M_PI/2;
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].position     = sin(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].speed        = cos(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].acceleration = -sin(t);
        joint_state.time = base::Time::now();
        robot_model.update(joint_state);

        for(size_t i = 0; i < joint_state.size(); i++){
            q_and_q_dot.q(i)    = joint_state[i].position;
            q_and_q_dot.qdot(i) = joint_state[i].speed;
        }

        KDL::FrameVel frame_vel;
        vel_solver.JntToCart(q_and_q_dot, frame_vel);

        base::samples::RigidBodyStateSE3 cstate;
        BOOST_CHECK_NO_THROW( cstate = robot_model.rigidBodyState(root, "kuka_lbr_l_tcp"));
        base::Vector3d euler = base::getEuler(cstate.pose.orientation);
        /*printf("Position:    %.4f %.4f %.4f\n",   cstate.pose.position(0), cstate.pose.position(1), cstate.pose.position(2));
        printf("Orientation: %.4f %.4f %.4f\n",   euler(0),                euler(1),                euler(2));
        printf("Linear Vel:  %.4f %.4f %.4f\n",   cstate.twist.linear(0),  cstate.twist.linear(1),  cstate.twist.linear(2));
        printf("Angular Vel: %.4f %.4f %.4f\n",   cstate.twist.angular(0), cstate.twist.angular(1), cstate.twist.angular(2));
        printf("Linear Acc:  %.4f %.4f %.4f\n",   cstate.acceleration.linear(0),  cstate.acceleration.linear(1),  cstate.acceleration.linear(2));
        printf("Angular Acc: %.4f %.4f %.4f\n\n", cstate.acceleration.angular(0), cstate.acceleration.angular(1), cstate.acceleration.angular(2));*/

        for(int i = 0; i < 3; i++){
            BOOST_CHECK(cstate.pose.position(i) == frame_vel.GetFrame().p(i));
            BOOST_CHECK(cstate.twist.linear(i) == frame_vel.deriv().vel(i));
            BOOST_CHECK(cstate.twist.angular(i) == frame_vel.deriv().rot(i));
        }
        double qx, qy, qz, qw;
        frame_vel.GetFrame().M.GetQuaternion(qx, qy, qz, qw);

        BOOST_CHECK(cstate.pose.orientation.x() == qx);
        BOOST_CHECK(cstate.pose.orientation.y() == qy);
        BOOST_CHECK(cstate.pose.orientation.z() == qz);
        BOOST_CHECK(cstate.pose.orientation.w() == qw);

        usleep(0.01*1000*1000);
        t+=0.01;
    }
}

BOOST_AUTO_TEST_CASE(floating_base_test)
{
    /**
     * Check whether the automatic configuration of a floating base in WBC works as intended. Compare FK with a URDF model
     * where the floating base is already integrated as virtual 6 DoF linkage.
     */

    srand(time(NULL));

    string urdf_filename = rootDir() + "/models/kuka/urdf/kuka_iiwa.urdf";
    string urdf_filename_floating_base = rootDir() + "/models/kuka/urdf/kuka_iiwa_with_floating_base.urdf";

    wbc::RobotModelKDL robot_model;
    vector<RobotModelConfig> configs;
    vector<string> actuated_joint_names;
    vector<string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    }
    RobotModelConfig config(urdf_filename, joint_names, actuated_joint_names, true);
    BOOST_CHECK(robot_model.configure(config) == true);

    // Check independent joints
    BOOST_CHECK(robot_model.noOfJoints() == joint_names.size());
    for(uint i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK(robot_model.jointNames()[i] == joint_names[i]);

    // Check actuated joints
    BOOST_CHECK(robot_model.noOfActuatedJoints() == actuated_joint_names.size());
    for(uint i = 0; i < robot_model.noOfActuatedJoints(); i++)
        BOOST_CHECK(robot_model.actuatedJointNames()[i] == actuated_joint_names[i]);

    wbc::RobotModelKDL robot_model_floating_base;
    RobotModelConfig config_floating_base(urdf_filename_floating_base, joint_names, joint_names, false);
    BOOST_CHECK(robot_model_floating_base.configure(config_floating_base) == true);

    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfActuatedJoints());
    joint_state.names = robot_model.actuatedJointNames();
    for(int i = 0; i < robot_model.noOfActuatedJoints(); i++){
        joint_state[i].position = double(rand())/RAND_MAX;
        joint_state[i].speed = double(rand())/RAND_MAX;
        joint_state[i].acceleration = double(rand())/RAND_MAX;
    }
    base::samples::RigidBodyStateSE3 floating_base_pose;
    floating_base_pose.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_pose.pose.orientation.setIdentity();
    floating_base_pose.twist.setZero();
    floating_base_pose.acceleration.setZero();

    base::samples::Joints joint_state_floating_base = joint_state;
    for(int i = 0; i < 3; i++){
        base::JointState js;
        js.position = floating_base_pose.pose.position[i];
        joint_state_floating_base.names.push_back(joint_names[i]);
        joint_state_floating_base.elements.push_back(js);
    }
    for(int i = 0; i < 3; i++){
        base::JointState js;
        js.position = 0;
        joint_state_floating_base.names.push_back(joint_names[i+3]);
        joint_state_floating_base.elements.push_back(js);
    }

    joint_state.time = joint_state_floating_base.time = floating_base_pose.time = base::Time::now();
    robot_model.update(joint_state, floating_base_pose);
    robot_model_floating_base.update(joint_state_floating_base);

    base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState("world", "kuka_lbr_l_tcp");
    base::samples::RigidBodyStateSE3 rbs_floating_base = robot_model_floating_base.rigidBodyState("world", "kuka_lbr_l_tcp");

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs.pose.position(i) - rbs_floating_base.pose.position(i)) < 1e-2);
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs.pose.orientation.coeffs()(i) - rbs_floating_base.pose.orientation.coeffs()(i)) < 1e-2);

}


