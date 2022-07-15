#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/kdl/KinematicChainKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "tools/URDFTools.hpp"
#include <regex>
#include <kdl_parser/kdl_parser.hpp>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configuration){

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

    // Valid config
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(robot_model.hasActuatedJoint(joint_names[i]));
        BOOST_CHECK(robot_model.hasJoint(joint_names[i]));
        BOOST_CHECK(robot_model.jointIndex(joint_names[i]) == i);
    }
    BOOST_CHECK(robot_model.baseFrame() == "kuka_lbr_l_link_0");
    BOOST_CHECK(robot_model.worldFrame() == "kuka_lbr_l_link_0");
    BOOST_CHECK(robot_model.hasLink("kuka_lbr_l_link_0"));

    // Invalid filename
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urd");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Empty filename
    config = RobotModelConfig("");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with joint names
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
    }

    // Valid config with joint names and actuated joint names
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
    }

    // Valid config with actuated joint names only
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.actuated_joint_names = joint_names;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
    }

    // Missing joint name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = {"kuka_lbr_l_joint_1",
                          "kuka_lbr_l_joint_2",
                          "kuka_lbr_l_joint_3",
                          "kuka_lbr_l_joint_4",
                          "kuka_lbr_l_joint_5",
                          "kuka_lbr_l_joint_6"};
    config.actuated_joint_names = config.joint_names;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Invalid joint name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
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
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    config.actuated_joint_names.pop_back();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Invalid actuated joint name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = config.joint_names;
    config.actuated_joint_names[6] = "kuka_lbr_l_joint_X";
    BOOST_CHECK(robot_model.configure(config) == false);

    // Valid config with floating base
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.floating_base_state.pose.fromTransform(Eigen::Affine3d::Identity());
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK((floating_base_names + joint_names)[i] == robot_model.jointNames()[i]);
    for(size_t i = 0; i < robot_model.noOfActuatedJoints(); i++)
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
    BOOST_CHECK(robot_model.floatingBaseState().pose.position == config.floating_base_state.pose.position);
    BOOST_CHECK(robot_model.floatingBaseState().pose.orientation.coeffs() == config.floating_base_state.pose.orientation.coeffs());

    // Valid config with floating base and joint names
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = floating_base_names + joint_names;
    config.actuated_joint_names = joint_names;
    config.floating_base_state.pose.fromTransform(Eigen::Affine3d::Identity());
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK((floating_base_names + joint_names)[i] == robot_model.jointNames()[i]);
    for(size_t i = 0; i < robot_model.noOfActuatedJoints(); i++)
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
    BOOST_CHECK(robot_model.floatingBaseState().pose.position == config.floating_base_state.pose.position);
    BOOST_CHECK(robot_model.floatingBaseState().pose.orientation.coeffs() == config.floating_base_state.pose.orientation.coeffs());

    // Config with invalid floating base name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = floating_base_names + joint_names;
    config.joint_names[0] = "floating_base_trans_";
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with missing floating base name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = floating_base_names + joint_names;
    config.joint_names.erase(config.joint_names.begin());
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with invalid floating base state
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = floating_base_names + joint_names;
    config.actuated_joint_names = joint_names;
    config.floating_base = true;
    config.floating_base_state.pose.orientation = base::Vector4d(1,1,1,1);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with blacklisted joints
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.joint_names.pop_back();
    config.actuated_joint_names = config.joint_names;
    config.joint_blacklist.push_back(joint_names[6]);
    config.floating_base = false;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK((config.joint_names)[i] == robot_model.jointNames()[i]);
    for(size_t i = 0; i < robot_model.noOfActuatedJoints(); i++)
        BOOST_CHECK(config.joint_names[i] == robot_model.actuatedJointNames()[i]);

    // Config with blacklisted joints and missing joint name
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.joint_names.pop_back();
    config.joint_names.pop_back();
    config.actuated_joint_names = config.joint_names;
    config.joint_blacklist.push_back(joint_names[6]);
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with invalid joints in blacklist
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.joint_blacklist.push_back("kuka_lbr_l_joint_X");
    BOOST_CHECK(robot_model.configure(config) == false);

    // Config with contact points
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.contact_points.names.push_back("kuka_lbr_l_tcp");
    config.contact_points.elements.push_back(1);
    config.joint_blacklist.clear();
    BOOST_CHECK(robot_model.configure(config) == true);

    // Config with invalid contact points
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    config.contact_points.names.push_back("XYZ");
    config.contact_points.elements.push_back(1);
    BOOST_CHECK(robot_model.configure(config) == false);

}

BOOST_AUTO_TEST_CASE(forward_kinematics){

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

    string urdf_model_file = "../../../../models/others/urdf/single_joint.urdf";

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

        base::VectorXd acceleration = robot_model.jacobianDot("base", "ee")*joint_vel +robot_model.spaceJacobian("base", "ee")*joint_acc;
        double expected_y_acc = -acc*cos(pos) +  vel*vel*sin(pos);
        double expected_z_acc = -acc*sin(pos) -vel*vel*cos(pos);
        /*printf("Expected Linear Acc:  %.4f %.4f %.4f\n",   zero, expected_y_acc, expected_z_acc);
        printf("Computed Linear Acc:  %.4f %.4f %.4f\n\n",   acceleration(0), acceleration(1), acceleration(2));*/
        BOOST_CHECK(fabs(zero - cstate.acceleration.linear(0)) <= 1e-7);
        BOOST_CHECK(fabs(expected_y_acc - cstate.acceleration.linear(1)) <= 1e-7);
        BOOST_CHECK(fabs(expected_z_acc - cstate.acceleration.linear(2)) <= 1e-7);

        /*printf("Expected Angular Acc:  %.4f %.4f %.4f\n",   acc, zero, zero);
        printf("Computed Angular Acc:  %.4f %.4f %.4f\n\n",   acceleration(3), acceleration(4), acceleration(5));*/
        BOOST_CHECK(fabs(acc - cstate.acceleration.angular(0)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.acceleration.angular(1)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.acceleration.angular(2)) <= 1e-7);

        //printf("...........................................................\n");
        usleep(0.1*1000*1000);
        t+=0.1;
    }
}

BOOST_AUTO_TEST_CASE(compare_wbc_with_kdl){

    /**
     * Compare forward kinematics for KDL-based robot model in WBC with pure KDL solution
     */
    string urdf_filename = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    RobotModelConfig config(urdf_filename);

    string root = "kuka_lbr_l_link_0";
    string tip  = "kuka_lbr_l_tcp";

    base::samples::Joints joint_state;
    vector<string> joint_names = URDFTools::jointNamesFromURDF(config.file);
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = j.acceleration = 0;

    RobotModelKDL robot_model;
    BOOST_CHECK(robot_model.configure(config) == true);

    KDL::Chain chain;
    KDL::Tree tree;
    BOOST_CHECK(kdl_parser::treeFromFile(urdf_filename,tree) == true);
    tree.getChain(root, tip, chain);

    KDL::ChainFkSolverVel_recursive vel_solver(chain);
    KDL::JntArrayVel q_and_q_dot(joint_names.size());
    base::VectorXd q(joint_names.size()), qd(joint_names.size()), qdd(joint_names.size());

    double t = 0;
    while(t < 1){
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_1")].position     = M_PI/2;
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].position     = sin(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].speed        = cos(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].acceleration = -sin(t);
        joint_state.time = base::Time::now();
        robot_model.update(joint_state);

        for(size_t i = 0; i < joint_state.size(); i++){
            q_and_q_dot.q(i)    = q[i]  = joint_state[i].position;
            q_and_q_dot.qdot(i) = qd[i] = joint_state[i].speed;
            qdd[i] = joint_state[i].acceleration;
        }

        KDL::FrameVel frame_vel;
        vel_solver.JntToCart(q_and_q_dot, frame_vel);

        base::MatrixXd jac_dot = robot_model.jacobianDot(root, tip);
        base::MatrixXd jac     = robot_model.spaceJacobian(root, tip);
        base::VectorXd acc     = jac_dot*qd + jac*qdd;

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
            BOOST_CHECK(cstate.pose.position(i)        == frame_vel.GetFrame().p(i));
            BOOST_CHECK(cstate.twist.linear(i)         == frame_vel.deriv().vel(i));
            BOOST_CHECK(cstate.twist.angular(i)        == frame_vel.deriv().rot(i));
            BOOST_CHECK(cstate.acceleration.linear(i)  == acc[i]);
            BOOST_CHECK(cstate.acceleration.angular(i) == acc[i+3]);
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

    string urdf_filename = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string urdf_filename_floating_base = "../../../../models/kuka/urdf/kuka_iiwa_with_floating_base.urdf";

    wbc::RobotModelKDL robot_model;
    vector<string> actuated_joint_names;
    vector<string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    }

    base::samples::RigidBodyStateSE3 floating_base_state;
    base::Vector3d euler(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.twist.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);

    RobotModelConfig config(urdf_filename, joint_names, actuated_joint_names, true);
    config.floating_base_state = floating_base_state;
    BOOST_CHECK(robot_model.configure(config) == true);
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

    base::samples::Joints joint_state_floating_base = joint_state;
    for(int i = 0; i < 3; i++){
        base::JointState js;
        js.position = floating_base_state.pose.position[i];
        js.speed = floating_base_state.twist.linear[i];
        js.acceleration = floating_base_state.acceleration.linear[i];
        joint_state_floating_base.names.push_back(joint_names[i]);
        joint_state_floating_base.elements.push_back(js);
    }
    for(int i = 0; i < 3; i++){
        base::JointState js;
        js.position = euler[i];
        js.speed = floating_base_state.twist.angular[i];
        js.acceleration = floating_base_state.acceleration.angular[i];
        joint_state_floating_base.names.push_back(joint_names[i+3]);
        joint_state_floating_base.elements.push_back(js);
    }

    joint_state.time = joint_state_floating_base.time = floating_base_state.time = base::Time::now();
    robot_model.update(joint_state, floating_base_state);
    robot_model_floating_base.update(joint_state_floating_base);

    base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState("world", "kuka_lbr_l_link_7");
    base::samples::RigidBodyStateSE3 rbs_floating_base = robot_model_floating_base.rigidBodyState("world", "kuka_lbr_l_link_7");

    // (1) Check whether the internal modeling of the floating base works as intended, i.e., compare the results from kuka_iiwa.urdf and kuka_iiwa_with_floating_base.urdf

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(rbs.pose.position(i) - rbs_floating_base.pose.position(i)) < 1e-2);
        BOOST_CHECK(fabs(rbs.twist.linear(i) - rbs_floating_base.twist.linear(i)) < 1e-2);
        BOOST_CHECK(fabs(rbs.twist.angular(i) - rbs_floating_base.twist.angular(i)) < 1e-2);
        BOOST_CHECK(fabs(rbs.acceleration.linear(i) - rbs_floating_base.acceleration.linear(i)) < 1e-2);
        BOOST_CHECK(fabs(rbs.acceleration.angular(i) - rbs_floating_base.acceleration.angular(i)) < 1e-2);
    }
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs.pose.orientation.coeffs()(i) - rbs_floating_base.pose.orientation.coeffs()(i)) < 1e-2);

    // (2) Check if the floating base state is correct in general, i.e., compare with the configured state

    base::samples::RigidBodyStateSE3 floating_base_state_measured = robot_model.rigidBodyState("world", "kuka_lbr_l_link_0");

    base::MatrixXd rot_mat = floating_base_state.pose.orientation.toRotationMatrix().inverse();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(floating_base_state_measured.pose.position(i) - floating_base_state.pose.position(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.twist.linear(i) - floating_base_state.twist.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.twist.angular(i) - (rot_mat*floating_base_state.twist.angular)(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.acceleration.linear(i) - floating_base_state.acceleration.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(floating_base_state_measured.acceleration.angular(i) - (rot_mat*floating_base_state.acceleration.angular)(i)) < 1e-3);
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

    // (3) Check if the floating base virtual joints are correctly set in the joint state

    base::samples::Joints full_joint_state = robot_model.jointState(joint_names);
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(full_joint_state[joint_names[i]].position - floating_base_state.pose.position[i]) < 1e-3);
        BOOST_CHECK(fabs(full_joint_state[joint_names[i]].speed - floating_base_state.twist.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(full_joint_state[joint_names[i]].acceleration - floating_base_state.acceleration.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(full_joint_state[joint_names[i+3]].position - euler[i]) < 1e-3);
        BOOST_CHECK(fabs(full_joint_state[joint_names[i+3]].speed - floating_base_state.twist.angular[i]) < 1e-3);
        BOOST_CHECK(fabs(full_joint_state[joint_names[i+3]].acceleration - floating_base_state.acceleration.angular[i]) < 1e-3);
    }

    /*cout<<"Measured joint state"<<endl;
    cout<<"Position:              "; for(int i = 0; i < 3; i++) cout << full_joint_state[i].position<<" ";   cout<<endl;
    cout<<"Orientation:           "; for(int i = 0; i < 3; i++) cout << full_joint_state[i+3].position<<" "; cout<<endl;
    cout<<"Twist (linear):        "; for(int i = 0; i < 3; i++) cout << full_joint_state[i].speed<<" "; cout<<endl;
    cout<<"Twist (linear):        "; for(int i = 0; i < 3; i++) cout << full_joint_state[i+3].speed<<" "; cout<<endl;
    cout<<"Acceleration (linear): "; for(int i = 0; i < 3; i++) cout << full_joint_state[i].acceleration<<" "; cout<<endl;
    cout<<"Acceleration (linear): "; for(int i = 0; i < 3; i++) cout << full_joint_state[i+3].acceleration<<" "; cout<<endl<<endl;*/
}

BOOST_AUTO_TEST_CASE(com_jacobian_test)
{
    /**
     * Check correctness of CoM Jacobian
     */

    srand(time(NULL));
    double dt = 0.002;
    
    string urdf_filename = "../../../../models/kuka/urdf/kuka_iiwa.urdf";

    wbc::RobotModelKDL robot_model;
    vector<string> actuated_joint_names;
    vector<string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    }
    RobotModelConfig config(urdf_filename, joint_names, actuated_joint_names, true);
    config.floating_base_state.pose.fromTransform(Eigen::Affine3d::Identity());
    BOOST_CHECK(robot_model.configure(config) == true);

    Eigen::VectorXd qd(joint_names.size());
    qd.head<3>() << 0.0, 0.0, 5.0;
    qd.segment<3>(3).setZero();

    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfActuatedJoints());
    joint_state.names = robot_model.actuatedJointNames();
    for(int i = 0; i < robot_model.noOfActuatedJoints(); i++){
        joint_state[i].position = double(rand()) / RAND_MAX;
        joint_state[i].speed = double(rand()) / RAND_MAX;
        joint_state[i].acceleration = 0; // double(rand()) / RAND_MAX;
        qd(i+6) = joint_state[i].speed;
    }
    base::samples::RigidBodyStateSE3 floating_base_pose;
    floating_base_pose.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_pose.pose.orientation.setIdentity();
    floating_base_pose.twist.setZero();
    floating_base_pose.twist.linear = qd.head<3>();
    floating_base_pose.acceleration.setZero();

    joint_state.time = floating_base_pose.time = base::Time::now();
    robot_model.update(joint_state, floating_base_pose);
    
    base::VectorXd com = robot_model.centerOfMass().pose.position;
    base::MatrixXd com_jacobian = robot_model.comJacobian(); 
    base::VectorXd com_vel = com_jacobian * qd;
    
    // integrate one step and update model. then compute COM and get COM velocity from differentiations
    floating_base_pose.pose.position = floating_base_pose.pose.position + dt * floating_base_pose.twist.linear;
    for(int i = 0; i < robot_model.noOfActuatedJoints(); i++)
        joint_state[i].position = joint_state[i].position + dt * joint_state[i].speed;

    robot_model.update(joint_state, floating_base_pose);

    base::VectorXd com_next = robot_model.centerOfMass().pose.position;
    base::Vector3d com_vel_diff = (1.0 / dt) * (com_next - com);

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(com_vel(i) - com_vel_diff(i)) < 1e-3);

    /*std::cout << "\n\nCOM Jacobian (" << com_jacobian.rows() << "x" << com_jacobian.cols() << "):" << std::endl << com_jacobian << std::endl;
    std::cout << "\nqd (size: " << qd.size() << "): " << qd.transpose() << std::endl;
    std::cout << "\nCOM velocity (from jacobian) = " << com_vel.transpose() << std::endl;
    std::cout << "COM velocity (from differentiation) = " << com_vel_diff.transpose() << std::endl;*/
    

}
