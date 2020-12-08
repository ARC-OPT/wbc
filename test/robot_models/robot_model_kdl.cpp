#include <boost/test/unit_test.hpp>
#include "robot_models/RobotModelKDL.hpp"
#include "robot_models/KinematicChainKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include "tools/URDFTools.hpp"

using namespace std;
using namespace wbc;
using namespace RigidBodyDynamics;

BOOST_AUTO_TEST_CASE(forward_kinematics_wbc_vs_kdl){

    string urdf_model_file = "../../../models/urdf/kuka/kuka_iiwa.urdf";

    base::samples::Joints joint_state;
    vector<string> joint_names = URDFTools::jointNamesFromURDF(urdf_model_file);
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    RobotModelKDL robot_model;
    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_model_file, joint_names, joint_names, false)) == true);

    KDL::Chain chain;
    BOOST_CHECK_NO_THROW(robot_model.getTree().getChain("kuka_lbr_l_link_0", "kuka_lbr_l_tcp", chain));

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
        BOOST_CHECK_NO_THROW( cstate = robot_model.rigidBodyState("kuka_lbr_l_link_0", "kuka_lbr_l_tcp"));
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

BOOST_AUTO_TEST_CASE(jacobian_and_forward_kinematics_wbc_vs_kdl){

    vector<string> joint_names;
    joint_names.push_back("base_to_rot");
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    string urdf_model_file = "../../../models/urdf/others/single_joint.urdf";

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

BOOST_AUTO_TEST_CASE(compare_kdl_vs_rbdl){

    srand(time(NULL));

    std::string urdf_filename = "../../../models/urdf/kuka/kuka_iiwa.urdf";

    // RBDL Robot model
    Model rbdl_model;
    BOOST_CHECK(Addons::URDFReadFromFile(urdf_filename.c_str(), &rbdl_model, false) == true);
    Eigen::VectorXd q(rbdl_model.dof_count), qdot(rbdl_model.dof_count);

    Eigen::MatrixXd H;
    H.resize(rbdl_model.dof_count,rbdl_model.dof_count);
    H.setZero();

    Eigen::VectorXd C;
    C.resize(rbdl_model.dof_count);

    // WBC (KDL) Robot Model
    wbc::RobotModelKDL robot_model;
    std::vector<std::string> joint_names;
    for(int i = 0; i < rbdl_model.dof_count; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));

    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_filename, joint_names, joint_names)) == true);
    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfJoints());
    joint_state.names = robot_model.jointNames();

    for(int n = 0; n < 100; n++){

        for(int i = 0; i < rbdl_model.dof_count; i++){
            q(i) = double(rand())/RAND_MAX;
            qdot(i) = double(rand())/RAND_MAX;
            base::JointState js;
            joint_state[i].position = q(i);
            joint_state[i].speed = qdot(i);
        }
        joint_state.time = base::Time::now();

        // Compute with RBDL

        base::Time start=base::Time::now();
        CompositeRigidBodyAlgorithm(rbdl_model, q, H);
        base::Time end=base::Time::now();
        double time_rbdl_joint_space_inertia_comp = (end-start).toSeconds();

        start=base::Time::now();
        NonlinearEffects(rbdl_model, q, qdot, C);
        end=base::Time::now();
        double time_rbdl_bias_torques = (end-start).toSeconds();

        int body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        base::Vector3d position_rbdl = CalcBodyToBaseCoordinates(rbdl_model,q,body_id,base::Vector3d(0,0,0));
        base::Matrix3d orientation_rbdl = CalcBodyWorldOrientation(rbdl_model,q,body_id).inverse();

        body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        Math::SpatialVector twist_rbdl = CalcPointVelocity6D(rbdl_model, q, qdot, body_id, base::Vector3d(0,0,0));

        body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        Math::MatrixNd jac_rbdl(6,rbdl_model.dof_count);
        CalcPointJacobian6D(rbdl_model, q, body_id, base::Vector3d(0,0,0), jac_rbdl);

        // Compute with WBC

        BOOST_CHECK_NO_THROW(robot_model.update(joint_state));

        start=base::Time::now();
        base::MatrixXd jnt_space_inertia_mat_wbc = robot_model.jointSpaceInertiaMatrix();
        end=base::Time::now();
        double time_wbc_joint_space_inertia_matrix = (end-start).toSeconds();

        start=base::Time::now();
        base::VectorXd bias_forces_wbc = robot_model.biasForces();
        end=base::Time::now();
        double time_wbc_bias_torques = (end-start).toSeconds();

        base::Vector3d position_wbc = robot_model.rigidBodyState("kuka_lbr_l_link_0", "kuka_lbr_l_link_7").pose.position;
        base::Matrix3d orientation_wbc = robot_model.rigidBodyState("kuka_lbr_l_link_0", "kuka_lbr_l_link_7").pose.orientation.toRotationMatrix();
        base::Twist twist_wbc = robot_model.rigidBodyState("kuka_lbr_l_link_0", "kuka_lbr_l_link_7").twist;
        base::MatrixXd jac_wbc = robot_model.spaceJacobian("kuka_lbr_l_link_0", "kuka_lbr_l_link_7");

        // Check joint space inertia matrix
        for(int i = 0; i < rbdl_model.dof_count; i++)
            for(int j = 0; j < rbdl_model.dof_count; j++)
                BOOST_CHECK(fabs(jnt_space_inertia_mat_wbc(i,j) - H(i,j)) < 1e-3);

        // Check bias torques
        for(int i = 0; i < rbdl_model.dof_count; i++)
            BOOST_CHECK(fabs(bias_forces_wbc[i] - C[i])  < 1e-6);

        // Check forward kinematics
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(position_wbc(i) - position_rbdl(i))  < 1e-9);
            for(int j = 0; j < 3; j++)
                BOOST_CHECK(fabs(orientation_wbc(i,j) - orientation_rbdl(i,j)) < 1e-9);
        }
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(twist_wbc.linear(i) - twist_rbdl(i+3))  < 1e-6);
            BOOST_CHECK(fabs(twist_wbc.angular(i) - twist_rbdl(i))  < 1e-6);
        }

        // Check Jacobian
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 7; j++)
                BOOST_CHECK(fabs(jac_rbdl(i+3,j) - jac_wbc(i,j)) < 1e-5);
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 7; j++)
                BOOST_CHECK(fabs(jac_rbdl(i,j) - jac_wbc(i+3,j)) < 1e-5);

        /*cout<<"Joint space inertia Matrix"<<endl;

        cout<<"RBDL "<<endl;
        cout<<H<<endl;
        cout<<"Computation Time "<<time_rbdl_joint_space_inertia_comp<<endl<<endl;
        cout<<"WBC "<<endl;
        cout<<jnt_space_inertia_mat_wbc<<endl;
        cout<<"Computation Time "<<time_wbc_joint_space_inertia_matrix<<endl<<endl;

        cout<<"Bias torques"<<endl;
        cout<<"RBDL"<<endl;
        cout<<C.transpose()<<endl;
        cout<<"Computation Time "<<time_rbdl_bias_torques<<endl<<endl;
        cout<<"WBC"<<endl;
        cout<<bias_forces_wbc.transpose()<<endl;
        cout<<"Computation Time "<<time_wbc_bias_torques<<endl<<endl;

        cout<<"Forward Kinematics"<<endl;
        cout<<"RBDL"<<endl;
        cout<<"Position:      "<<position_rbdl.transpose()<<endl;
        cout<<"Orientation:   "<<base::Quaterniond(orientation_rbdl).coeffs().transpose()<<endl;
        cout<<"Twist linear:  "<<twist_rbdl.segment(3,3).transpose()<<endl;
        cout<<"Twist angular: "<<twist_rbdl.segment(0,3).transpose()<<endl<<endl;
        cout<<"WBC"<<endl;
        cout<<"Position:      "<<position_wbc.transpose()<<endl;
        cout<<"Orientation:   "<<base::Quaterniond(orientation_wbc).coeffs().transpose()<<endl;
        cout<<"Twist linear:  "<<twist_wbc.linear.transpose()<<endl;
        cout<<"Twist angular: "<<twist_wbc.angular.transpose()<<endl<<endl;

        cout<<"Jacobian "<<endl;
        cout<<"RBDL"<<endl;
        cout<<jac_rbdl<<endl;
        cout<<"WBC: "<<endl;
        cout<<jac_wbc<<endl<<endl;

        cout<<"-------------------------------------------------"<<endl<<endl;*/
    }
}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_rbdl_floating_base){

    srand(time(NULL));

    // RBDL Robot model

    std::string urdf_filename = "../../../models/urdf/kuka/kuka_iiwa.urdf";
    Model rbdl_model;
    BOOST_CHECK(Addons::URDFReadFromFile(urdf_filename.c_str(), &rbdl_model, true) == true);

    Eigen::MatrixXd H;
    H.resize(rbdl_model.dof_count,rbdl_model.dof_count);
    H.setZero();

    Eigen::VectorXd C;
    C.resize(rbdl_model.dof_count);

    // IMPORTANT: RBDL adds the real part of the quaternion for the floating base at the end of the state vector, so we have to augment the state vector by one here!
    Eigen::VectorXd q(rbdl_model.dof_count+1), qdot(rbdl_model.dof_count);

    // WBC (KDL) Robot Model

    wbc::RobotModelKDL robot_model;
    std::vector<std::string> actuated_joint_names;
    std::vector<std::string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                           "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));
    }

    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_filename, joint_names, actuated_joint_names, true)) == true);
    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfActuatedJoints());
    joint_state.names = robot_model.actuatedJointNames();

    for(int n = 0; n < 1; n++){
        for(int i = 0; i < rbdl_model.q_size; i++)
            q(i) = double(rand())/RAND_MAX;
        for(int i = 0; i < rbdl_model.qdot_size; i++)
            qdot(i) = double(rand())/RAND_MAX;

        for(int i = 0; i < robot_model.noOfActuatedJoints(); i++){
            joint_state[i].position = q(i+6);
            joint_state[i].speed = qdot(i+6);
        }
        joint_state.time = base::Time::now();

        int floating_body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_0");
        base::Quaterniond init_orientation = base::AngleAxisd(double(rand())/RAND_MAX, base::Vector3d::UnitX()) *
                                             base::AngleAxisd(double(rand())/RAND_MAX, base::Vector3d::UnitY()) *
                                             base::AngleAxisd(double(rand())/RAND_MAX, base::Vector3d::UnitZ());

        rbdl_model.SetQuaternion(floating_body_id, Math::Quaternion(init_orientation.coeffs()), q);

        // NOTE: RBDL uses a different representation of the floating base (spherical joint instead of 3 serial rotational joints)
        // So we only get consistent results if the rotation (and rotational velocity) is zero
        q(3)=q(4)=q(5)=0;
        q(rbdl_model.q_size-1)=1;
        qdot(3)=qdot(4)=qdot(5)=0;

        // Compute with RBDL

        base::Time start=base::Time::now();
        CompositeRigidBodyAlgorithm(rbdl_model, q, H);
        base::Time end=base::Time::now();
        double time_rbdl_joint_space_inertia_comp = (end-start).toSeconds();

        start=base::Time::now();
        NonlinearEffects(rbdl_model, q, qdot, C);
        end=base::Time::now();
        double time_rbdl_bias_torques = (end-start).toSeconds();

        int body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        base::Vector3d position_rbdl = CalcBodyToBaseCoordinates(rbdl_model,q,body_id,base::Vector3d(0,0,0));
        base::Matrix3d orientation_rbdl = CalcBodyWorldOrientation(rbdl_model,q,body_id).inverse();

        body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        Math::SpatialVector twist_rbdl = CalcPointVelocity6D(rbdl_model, q, qdot, body_id, base::Vector3d(0,0,0));

        body_id = rbdl_model.GetBodyId("kuka_lbr_l_link_7");
        Math::MatrixNd jac_rbdl(6,rbdl_model.dof_count);
        CalcPointJacobian6D(rbdl_model, q, body_id, base::Vector3d(0,0,0), jac_rbdl);

        // Compute with WBC (KDL)

        base::samples::RigidBodyStateSE3 floating_rbs;
        floating_rbs.time = base::Time::now();
        floating_rbs.pose.position = base::Vector3d(q(0),q(1),q(2));
        floating_rbs.pose.orientation = base::Quaterniond(q(rbdl_model.q_size-1),q(3),q(4),q(5));
        floating_rbs.twist = base::Twist(base::Vector3d(qdot(0),qdot(1),qdot(2)), base::Vector3d(qdot(3),qdot(4),qdot(5)));
        floating_rbs.acceleration.setZero();
        BOOST_CHECK_NO_THROW(robot_model.update(joint_state, floating_rbs));

        base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState("world", "kuka_lbr_l_link_7");
        base::Vector3d position_wbc = rbs.pose.position;
        base::Matrix3d orientation_wbc = rbs.pose.orientation.toRotationMatrix();
        base::Twist twist_wbc = rbs.twist;
        base::MatrixXd jac_wbc = robot_model.spaceJacobian("world", "kuka_lbr_l_link_7");

        start = base::Time::now();
        base::MatrixXd jnt_space_inertia_mat_wbc = robot_model.jointSpaceInertiaMatrix();
        end = base::Time::now();
        double time_wbc_joint_space_inertia_matrix = (end-start).toSeconds();

        start = base::Time::now();
        base::VectorXd bias_forces_wbc = robot_model.biasForces();
        end = base::Time::now();
        double time_wbc_bias_torques = (end-start).toSeconds();

        /*cout<<"RBDL "<<endl;
        cout<<H<<endl;
        cout<<"Computation Time "<<time_rbdl_joint_space_inertia_comp<<endl<<endl;
        cout<<"WBC "<<endl;
        cout<<jnt_space_inertia_mat_wbc<<endl;
        cout<<"Computation Time "<<time_wbc_joint_space_inertia_matrix<<endl<<endl;

        cout<<"Bias torques"<<endl;
        cout<<"RBDL"<<endl;
        cout<<C.transpose()<<endl;
        cout<<"Computation Time "<<time_rbdl_bias_torques<<endl<<endl;
        cout<<"WBC"<<endl;
        cout<<bias_forces_wbc.transpose()<<endl;
        cout<<"Computation Time "<<time_wbc_bias_torques<<endl<<endl;

        cout<<"Forward Kinematics"<<endl;
        cout<<"RBDL"<<endl;
        cout<<"Position:      "<<position_rbdl.transpose()<<endl;
        cout<<"Orientation:   "<<base::Quaterniond(orientation_rbdl).coeffs().transpose()<<endl;
        cout<<"Twist linear:  "<<twist_rbdl.segment(3,3).transpose()<<endl;
        cout<<"Twist angular: "<<twist_rbdl.segment(0,3).transpose()<<endl<<endl;
        cout<<"WBC"<<endl;
        cout<<"Position:      "<<position_wbc.transpose()<<endl;
        cout<<"Orientation:   "<<base::Quaterniond(orientation_wbc).coeffs().transpose()<<endl;
        cout<<"Twist linear:  "<<twist_wbc.linear.transpose()<<endl;
        cout<<"Twist angular: "<<twist_wbc.angular.transpose()<<endl<<endl;

        cout<<"Jacobian "<<endl;
        cout<<"RBDL"<<endl;
        cout<<jac_rbdl<<endl;
        cout<<"WBC: "<<endl;
        cout<<jac_wbc<<endl<<endl;

        cout<<"-------------------------------------------------"<<endl<<endl;*/

        // Check joint space inertia matrix
        for(int i = 0; i < rbdl_model.dof_count; i++)
            for(int j = 0; j < rbdl_model.dof_count; j++)
                BOOST_CHECK(fabs(jnt_space_inertia_mat_wbc(i,j) - H(i,j)) < 1e-3);

        // Check bias torques
        for(int i = 0; i < rbdl_model.dof_count; i++)
            BOOST_CHECK(fabs(bias_forces_wbc[i] - C[i])  < 1e-3);

        // Check forward kinematics
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(position_wbc(i) - position_rbdl(i))  < 1e-3);
            for(int j = 0; j < 3; j++)
                BOOST_CHECK(fabs(orientation_wbc(i,j) - orientation_rbdl(i,j)) < 1e-3);
        }
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(twist_wbc.linear(i) - twist_rbdl(i+3))  < 1e-3);
            BOOST_CHECK(fabs(twist_wbc.angular(i) - twist_rbdl(i))  < 1e-3);
        }

        // Check Jacobian
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 7; j++)
                BOOST_CHECK(fabs(jac_rbdl(i+3,j) - jac_wbc(i,j)) < 1e-3);
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 7; j++)
                BOOST_CHECK(fabs(jac_rbdl(i,j) - jac_wbc(i+3,j)) < 1e-3);
    }
}

BOOST_AUTO_TEST_CASE(floating_base_test)
{
    srand(time(NULL));

    std::string urdf_filename = "../../../models/urdf/kuka/kuka_iiwa.urdf";
    std::string urdf_filename_floating_base = "../../../models/urdf/kuka/kuka_iiwa_with_floating_base.urdf";

    wbc::RobotModelKDL robot_model;
    vector<RobotModelConfig> configs;
    std::vector<std::string> actuated_joint_names;
    std::vector<std::string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z", "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));
    }
    RobotModelConfig config(urdf_filename, joint_names, actuated_joint_names, true);
    BOOST_CHECK(robot_model.configure(config) == true);

    wbc::RobotModelKDL robot_model_floating_base;
    config.file = urdf_filename_floating_base;
    config.floating_base = false;
    BOOST_CHECK(robot_model_floating_base.configure(config) == true);

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
    }for(int i = 0; i < 3; i++){
        base::JointState js;
        js.position = 0;
        joint_state_floating_base.names.push_back(joint_names[i+3]);
        joint_state_floating_base.elements.push_back(js);
    }

    joint_state.time = joint_state_floating_base.time = base::Time::now();
    robot_model.update(joint_state, floating_base_pose);
    robot_model_floating_base.update(joint_state_floating_base);

    base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState("world", "kuka_lbr_l_tcp");
    base::samples::RigidBodyStateSE3 rbs_floating_base = robot_model_floating_base.rigidBodyState("world", "kuka_lbr_l_tcp");

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs.pose.position(i) - rbs_floating_base.pose.position(i)) < 1e-3);
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs.pose.orientation.coeffs()(i) - rbs_floating_base.pose.orientation.coeffs()(i)) < 1e-3);

}


