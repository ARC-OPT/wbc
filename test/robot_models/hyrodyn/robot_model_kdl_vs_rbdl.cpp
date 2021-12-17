#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "robot_models/kdl/KinematicChainKDL.hpp"
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

string rootDir(){
    std::string root_dir = string(__FILE__);
    const size_t last_slash_idx = root_dir.rfind('/');
    root_dir =  root_dir.substr(0, last_slash_idx) + "/../../..";
    return root_dir;
}

BOOST_AUTO_TEST_CASE(compare_kdl_vs_rbdl){

    /**
     * Compare kinematics and dynamics of the KDL-based robot model in WBC with the rigid body dynamics library (RBDL)
     */

    srand(time(NULL));

    string urdf_filename = rootDir() + "/models/kuka/urdf/kuka_iiwa.urdf";
    string root = "kuka_lbr_l_link_0";
    string tip  = "kuka_lbr_l_link_7";

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
    vector<string> joint_names;
    for(int i = 0; i < rbdl_model.dof_count; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));

    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_filename, joint_names, joint_names)) == true);
    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfJoints());
    joint_state.names = robot_model.jointNames();

    for(int n = 0; n < 10; n++){

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

        int body_id = rbdl_model.GetBodyId(tip.c_str());
        base::Vector3d position_rbdl = CalcBodyToBaseCoordinates(rbdl_model,q,body_id,base::Vector3d(0,0,0));
        base::Matrix3d orientation_rbdl = CalcBodyWorldOrientation(rbdl_model,q,body_id).inverse();

        Math::SpatialVector twist_rbdl = CalcPointVelocity6D(rbdl_model, q, qdot, body_id, base::Vector3d(0,0,0));

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

        base::Vector3d position_wbc = robot_model.rigidBodyState(root, tip).pose.position;
        base::Matrix3d orientation_wbc = robot_model.rigidBodyState(root, tip).pose.orientation.toRotationMatrix();
        base::Twist twist_wbc = robot_model.rigidBodyState(root, tip).twist;
        base::MatrixXd jac_wbc = robot_model.spaceJacobian(root, tip);

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

BOOST_AUTO_TEST_CASE(compare_wbc_vs_rbdl_floating_base){

    /**
     * Compare kinematics and dynamics of WBC for a floating base robot with the rigid body dynamics library (RBDL)
     */

    string urdf_filename = rootDir() + "/models/kuka/urdf/kuka_iiwa.urdf";
    string world = "world";
    string root = "kuka_lbr_l_link_0";
    string tip  = "kuka_lbr_l_tcp";
    srand(time(NULL));

    // RBDL Robot model

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
    vector<string> actuated_joint_names;
    vector<string> joint_names ={"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                 "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    for(int i = 0; i < 7; i++){
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    }

    BOOST_CHECK(robot_model.configure(RobotModelConfig(urdf_filename, joint_names, actuated_joint_names, true)) == true);
    base::samples::Joints joint_state;
    joint_state.resize(robot_model.noOfActuatedJoints());
    joint_state.names = robot_model.actuatedJointNames();

    for(int n = 0; n < 10; n++){
        for(int i = 0; i < rbdl_model.q_size; i++)
            q(i) = double(rand())/RAND_MAX;
        for(int i = 0; i < rbdl_model.qdot_size; i++)
            qdot(i) = double(rand())/RAND_MAX;

        for(int i = 0; i < robot_model.noOfActuatedJoints(); i++){
            joint_state[i].position = q(i+6);
            joint_state[i].speed = qdot(i+6);
        }
        joint_state.time = base::Time::now();

        int floating_body_id = rbdl_model.GetBodyId(root.c_str());
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

        int body_id = rbdl_model.GetBodyId(tip.c_str());
        base::Vector3d position_rbdl = CalcBodyToBaseCoordinates(rbdl_model,q,body_id,base::Vector3d(0,0,0));
        base::Matrix3d orientation_rbdl = CalcBodyWorldOrientation(rbdl_model,q,body_id).inverse();

        Math::SpatialVector twist_rbdl = CalcPointVelocity6D(rbdl_model, q, qdot, body_id, base::Vector3d(0,0,0));

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

        base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState(world, tip);
        base::Vector3d position_wbc = rbs.pose.position;
        base::Matrix3d orientation_wbc = rbs.pose.orientation.toRotationMatrix();
        base::Twist twist_wbc = rbs.twist;
        base::MatrixXd jac_wbc = robot_model.spaceJacobian(world, tip);

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
