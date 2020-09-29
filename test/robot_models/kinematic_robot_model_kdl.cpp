#include <boost/test/unit_test.hpp>
#include "robot_models/KinematicRobotModelKDL.hpp"
#include "robot_models/KinematicChainKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>

using namespace std;
using namespace wbc;
using namespace RigidBodyDynamics;

BOOST_AUTO_TEST_CASE(cartesian_state){

    cout<<"\n----------------------- Testing Kinematic Chain KDL ----------------------"<<endl<<endl;

    vector<string> joint_names;
    for(int i = 0; i < 7; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    string urdf_model_file = "../../../test/data/kuka_lbr.urdf";

    KinematicRobotModelKDL robot_model;
    BOOST_CHECK(robot_model.configure(urdf_model_file) == true);

    KDL::Chain chain;
    BOOST_CHECK_NO_THROW(robot_model.getTree().getChain("kuka_lbr_center", "kuka_lbr_l_tcp", chain));

    KDL::ChainFkSolverVel_recursive vel_solver(chain);
    KDL::JntArrayVel q_and_q_dot(joint_names.size());

    double t = 0;
    while(t < 10){
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_1")].position     = M_PI/2;
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].position     = sin(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].speed        = cos(t);
        joint_state[joint_state.mapNameToIndex("kuka_lbr_l_joint_4")].acceleration = -sin(t);
        joint_state.time = base::Time::now();
        BOOST_CHECK_NO_THROW(robot_model.update(joint_state));

        for(size_t i = 0; i < joint_state.size(); i++){
            q_and_q_dot.q(i)    = joint_state[i].position;
            q_and_q_dot.qdot(i) = joint_state[i].speed;
        }

        KDL::FrameVel frame_vel;
        vel_solver.JntToCart(q_and_q_dot, frame_vel);

        base::samples::RigidBodyStateSE3 cstate;
        BOOST_CHECK_NO_THROW( cstate = robot_model.rigidBodyState("kuka_lbr_center", "kuka_lbr_l_tcp"));
        base::Vector3d euler = base::getEuler(cstate.pose.orientation);
        printf("Position:    %.4f %.4f %.4f\n",   cstate.pose.position(0), cstate.pose.position(1), cstate.pose.position(2));
        printf("Orientation: %.4f %.4f %.4f\n",   euler(0),                euler(1),                euler(2));
        printf("Linear Vel:  %.4f %.4f %.4f\n",   cstate.twist.linear(0),  cstate.twist.linear(1),  cstate.twist.linear(2));
        printf("Angular Vel: %.4f %.4f %.4f\n",   cstate.twist.angular(0), cstate.twist.angular(1), cstate.twist.angular(2));
        printf("Linear Acc:  %.4f %.4f %.4f\n",   cstate.acceleration.linear(0),  cstate.acceleration.linear(1),  cstate.acceleration.linear(2));
        printf("Angular Acc: %.4f %.4f %.4f\n\n", cstate.acceleration.angular(0), cstate.acceleration.angular(1), cstate.acceleration.angular(2));

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

BOOST_AUTO_TEST_CASE(jacobian_and_cartesian_state){

    cout<<"\n----------------------- Testing Kinematic Chain KDL ----------------------"<<endl<<endl;

    vector<string> joint_names;
    joint_names.push_back("base_to_rot");
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(base::JointState& j : joint_state.elements)
        j.position = j.speed = 0;

    string urdf_model_file = string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/simple_model.urdf";

    KinematicRobotModelKDL robot_model;
    robot_model.configure(urdf_model_file);

    double t = 0;
    base::VectorXd joint_vel(joint_state.size());
    base::VectorXd joint_acc(joint_state.size());
    while(t < 10){
        joint_state[joint_state.mapNameToIndex("base_to_rot")].position     = sin(t);
        joint_state[joint_state.mapNameToIndex("base_to_rot")].speed        = cos(t);
        joint_state[joint_state.mapNameToIndex("base_to_rot")].acceleration = -sin(t);
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_vel(i) = joint_state[i].speed;
            joint_acc(i) = joint_state[i].acceleration;
        }
        joint_state.time = base::Time::now();
        robot_model.update(joint_state);
        cout<<"Robot model update took "<<(base::Time::now() - joint_state.time).toSeconds()*1000<<" ms"<<endl<<endl;

        base::samples::RigidBodyStateSE3 cstate = robot_model.rigidBodyState("base", "ee");
        base::Vector3d euler = base::getEuler(cstate.pose.orientation);

        double zero = 0.0;
        double pos = joint_state[joint_state.mapNameToIndex("base_to_rot")].position;
        double vel = joint_state[joint_state.mapNameToIndex("base_to_rot")].speed;
        double acc = joint_state[joint_state.mapNameToIndex("base_to_rot")].acceleration;

        printf("Expected position: %.4f %.4f %.4f\n",   zero, -sin(pos), cos(pos));
        printf("Computed Position: %.4f %.4f %.4f\n\n",   cstate.pose.position(0), cstate.pose.position(1), cstate.pose.position(2));
        BOOST_CHECK(fabs(zero - cstate.pose.position(0)) <= 1e-7);
        BOOST_CHECK(fabs(-sin(pos) - cstate.pose.position(1)) <= 1e-7);
        BOOST_CHECK(fabs(cos(pos) - cstate.pose.position(2)) <= 1e-7);

        printf("Expected Orientation: %.4f %.4f %.4f\n",   zero, zero, pos);
        printf("Computed Orientation: %.4f %.4f %.4f\n\n",   euler(0),  euler(1), euler(2));
        BOOST_CHECK(fabs(zero -  euler(0)) <= 1e-7);
        BOOST_CHECK(fabs(zero - euler(1)) <= 1e-7);
        BOOST_CHECK(fabs(pos - euler(2)) <= 1e-7);

        printf("Expected Linear Vel:  %.4f %.4f %.4f\n",   zero, -vel*cos(pos), -vel*sin(pos));
        printf("Computed Linear Vel:  %.4f %.4f %.4f\n\n",   cstate.twist.linear(0), cstate.twist.linear(1), cstate.twist.linear(2));
        BOOST_CHECK(fabs(zero - cstate.twist.linear(0)) <= 1e-7);
        BOOST_CHECK(fabs(-vel*cos(pos) - cstate.twist.linear(1)) <= 1e-7);
        BOOST_CHECK(fabs(-vel*sin(pos) - cstate.twist.linear(2)) <= 1e-7);

        printf("Expected Angular Vel:  %.4f %.4f %.4f\n",   vel, zero, zero);
        printf("Computed Angular Vel:  %.4f %.4f %.4f\n\n",   cstate.twist.angular(0), cstate.twist.angular(1), cstate.twist.angular(2));
        BOOST_CHECK(fabs(vel - cstate.twist.angular(0)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.twist.angular(1)) <= 1e-7);
        BOOST_CHECK(fabs(zero - cstate.twist.angular(2)) <= 1e-7);

        base::VectorXd twist = robot_model.jacobian("base", "ee") * joint_vel;
        printf("Linear Vel from Jacobian: %.4f %.4f %.4f\n", twist(0), twist(1), twist(2));
        printf("Angular Vel from Jacobian: %.4f %.4f %.4f\n\n", twist(3), twist(4), twist(5));
        for(int i = 0; i < 3; i++){
            BOOST_CHECK(fabs(twist(i) - cstate.twist.linear(i)) <= 1e-7);
            BOOST_CHECK(fabs(twist(i+3) - cstate.twist.angular(i)) <= 1e-7);
        }

        base::VectorXd acceleration = robot_model.jacobianDot("base", "ee")*joint_vel + robot_model.jacobian("base", "ee")*joint_acc;

        double expected_y_acc = -acc*cos(pos) +  vel*vel*sin(pos);
        double expected_z_acc = -acc*sin(pos) -vel*vel*cos(pos);
        printf("Expected Linear Acc:  %.4f %.4f %.4f\n",   zero, expected_y_acc, expected_z_acc);
        printf("Computed Linear Acc:  %.4f %.4f %.4f\n\n",   acceleration(0), acceleration(1), acceleration(2));
        BOOST_CHECK(fabs(zero - acceleration(0)) <= 1e-7);
        BOOST_CHECK(fabs(expected_y_acc - acceleration(1)) <= 1e-7);
        BOOST_CHECK(fabs(expected_z_acc - acceleration(2)) <= 1e-7);

        printf("Expected Angular Acc:  %.4f %.4f %.4f\n",   acc, zero, zero);
        printf("Computed Angular Acc:  %.4f %.4f %.4f\n\n",   acceleration(3), acceleration(4), acceleration(5));
        BOOST_CHECK(fabs(acc - acceleration(3)) <= 1e-7);
        BOOST_CHECK(fabs(zero - acceleration(4)) <= 1e-7);
        BOOST_CHECK(fabs(zero - acceleration(5)) <= 1e-7);


        printf("...........................................................\n");
        usleep(0.1*1000*1000);
        t+=0.1;
    }
}

BOOST_AUTO_TEST_CASE(multi_robot){

    srand(time(NULL));

    vector<string> actuated_joint_names;
    for(int i = 0; i < 7; i++)
        actuated_joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));
    vector<string> all_joint_names = actuated_joint_names;
    const std::string virtual_joint_names[6] = {"_x", "_y", "_z", "_rot_x", "_rot_y", "_rot_z"};
    for(int i = 0; i < 6; i++)
        all_joint_names.push_back("object" + virtual_joint_names[i]);

    cout<<"Testing Model Creation .."<<endl<<endl;
    base::samples::Joints joint_state;
    joint_state.resize(actuated_joint_names.size());
    joint_state.names = actuated_joint_names;
    for(int i = 0; i < 7; i++)
        joint_state[i].position = rand()/(double)RAND_MAX;
    joint_state.time = base::Time::now();
    base::Pose initial_object_pose;
    initial_object_pose.position = base::Vector3d(0,0,0);
    initial_object_pose.orientation.setIdentity();

    KinematicRobotModelKDL robot_model;
    vector<RobotModelConfig> config(2);
    config[0].file = "../../../test/data/kuka_lbr.urdf";
    config[0].joint_names = joint_state.names;
    config[1].file = "../../../test/data/object.urdf";
    config[1].hook = "kuka_lbr_top_left_camera";
    config[1].initial_pose = initial_object_pose;
    BOOST_CHECK_EQUAL(robot_model.configure(config), true);
    BOOST_CHECK_EQUAL(robot_model.noOfActuatedJoints(), actuated_joint_names.size());
    BOOST_CHECK_EQUAL(robot_model.noOfJoints(), all_joint_names.size());
    for(int i = 0; i < robot_model.noOfActuatedJoints(); i++)
        BOOST_CHECK_EQUAL(robot_model.actuatedJointNames()[i], actuated_joint_names[i]);
    for(int i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK_EQUAL(robot_model.jointNames()[i], all_joint_names[i]);

    cout<<"Testing Model Update ...."<<endl<<endl;

    base::samples::RigidBodyStateSE3 object_pose;
    object_pose.pose.position = base::Vector3d(0,1,0);
    object_pose.pose.orientation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::Unit(2));
    base::NamedVector<base::samples::RigidBodyStateSE3> poses;
    poses.elements.push_back(object_pose);
    poses.names.push_back("object");
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state, poses););

    cout<<"Testing FK ..."<<endl<<endl;

    KDL::JntArray joint_positions(joint_state.size() + 6);
    base::Vector3d euler = base::getEuler(object_pose.pose.orientation);
    for(size_t i = 0; i < joint_state.size(); i++)
        joint_positions(6-i) = joint_state[i].position;
    for(size_t i = 0; i < 3; i++){
        joint_positions(i+7) = object_pose.pose.position(i);
        joint_positions(i+7+3) = euler(i);
    }

    KDL::Chain chain;
    BOOST_CHECK(robot_model.getTree().getChain("kuka_lbr_l_tcp", "object", chain) == true);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame pose_kdl;
    BOOST_CHECK_EQUAL(fk_solver.JntToCart(joint_positions, pose_kdl),0);

    cout<<"Pose from KDL: "<<endl;
    cout<<"x: "<<pose_kdl.p(0)<<" y: "<<pose_kdl.p(1)<<" z: "<<pose_kdl.p(2)<<endl;
    double x,y,z,w;
    pose_kdl.M.GetQuaternion(x,y,z,w);
    cout<<"qx: "<<x<<" qy: "<<y<<" qz: "<<z<<" qw: "<<w<<endl<<endl;

    base::samples::RigidBodyStateSE3 pose_from_model;
    BOOST_CHECK_NO_THROW(pose_from_model = robot_model.rigidBodyState("kuka_lbr_l_tcp", "object"););

    for(int i = 0; i < 3; i++)
        BOOST_CHECK_EQUAL(pose_kdl.p(i), pose_from_model.pose.position(i));
    BOOST_CHECK_EQUAL(pose_from_model.pose.orientation.x(),x);
    BOOST_CHECK_EQUAL(pose_from_model.pose.orientation.y(),y);
    BOOST_CHECK_EQUAL(pose_from_model.pose.orientation.z(),z);
    BOOST_CHECK_EQUAL(pose_from_model.pose.orientation.w(),w);

    cout<<"Pose from robot model "<<endl;
    cout<<"x: "<<pose_from_model.pose.position[0]<<" y: "<<pose_from_model.pose.position[1]<<" z: "<<pose_from_model.pose.position[2]<<endl;
    cout<<"qx: "<<pose_from_model.pose.orientation.x()<<" qy: "<<pose_from_model.pose.orientation.y()<<" qz: "<<pose_from_model.pose.orientation.z()<<" qw: "<<pose_from_model.pose.orientation.w()<<endl<<endl;

    cout<<"Testing Jacobian ..."<<endl<<endl;
    KDL::Jacobian jac_kdl(robot_model.noOfJoints());
    KDL::ChainJntToJacSolver jac_solver(chain);
    BOOST_CHECK_EQUAL(jac_solver.JntToJac(joint_positions, jac_kdl),0);

    base::MatrixXd jac = robot_model.jacobian("kuka_lbr_l_tcp", "object");

    cout<<"Jacobian from KDL"<<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"Jacobian from model"<<endl;
    cout<<jac<<endl<<endl;

    for(int i = 0; i < jac.rows(); i++)
        for(int j = 0; j < robot_model.noOfJoints(); j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));
}


BOOST_AUTO_TEST_CASE(compare_kdl_vs_rbdl){
    for(int n = 0; n < 1; n++){
        std::string urdf_filename = string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/kuka_iiwa.urdf";
        Model model;
        BOOST_CHECK(Addons::URDFReadFromFile(urdf_filename.c_str(), &model, true) == true);
        Eigen::VectorXd q(model.dof_count), qdot(model.dof_count);
        for(int i = 0; i < model.dof_count; i++)
            q(i) = 0;//double(rand())/RAND_MAX;
        q(5) = 0.5;
        std::cout<<"q"<<std::endl;
        for(int i = 0; i < q.size(); i++)
            std::cout<<q(i)<<std::endl;

        qdot.setZero();
        Eigen::MatrixXd H;
        H.resize(model.dof_count,model.dof_count);
        H.setZero();
        base::Time start=base::Time::now();
        CompositeRigidBodyAlgorithm(model, q, H);
        base::Time end=base::Time::now();
        cout<<"RBDL Joint Space Inertia Matrix: "<<endl;
        cout<<H<<endl;
        cout<<"Computation Time: "<<(end-start).toSeconds()*1000<<"ms"<<endl;

        Eigen::VectorXd C;
        C.resize(model.dof_count);
        start=base::Time::now();
        NonlinearEffects(model, q, qdot, C);
        end=base::Time::now();
        cout<<"RBDL Bias torques: ";
        cout<<C.transpose()<<endl;
        cout<<"Computation Time: "<<(end-start).toSeconds()*1000<<"ms"<<endl;

        wbc::KinematicRobotModelKDL robot_model;
        std::vector<std::string> joint_names;
        for(int i = 0; i < 7; i++)
            joint_names.push_back("kuka_lbr_l_joint_" + std::to_string(i+1));

        RobotModelConfig config;
        config.hook = "world";
        config.initial_pose.position.setZero();
        config.initial_pose.orientation.setIdentity();
        config.file = urdf_filename;
        config.joint_names = joint_names;
        std::vector<RobotModelConfig> configs;
        configs.push_back(config);
        if(!robot_model.configure(configs, true)){
            cerr << "Error loading robot model from urdf" << endl;
            abort();
        }
        cout<<"WBC Joint Space Inertia Matrix: "<<endl;
        base::samples::Joints joint_state;
        joint_state.resize(robot_model.noOfJoints());
        joint_state.names = robot_model.jointNames();
        for(int i = 0; i < robot_model.noOfJoints(); i++){
            base::JointState js;
            js.position = q(i);
            js.speed = 0;
            joint_state[i] = js;
        }
        joint_state.time = base::Time::now();
        robot_model.update(joint_state);

        std::cout<<"Joint state"<<std::endl;
        for(auto s : robot_model.jointState(robot_model.jointNames()).elements)
            std::cout<<s.position<<std::endl;


        start=base::Time::now();
        robot_model.computeJointSpaceInertiaMatrix();
        end=base::Time::now();
        cout<<robot_model.jointSpaceInertiaMatrix()<<endl;
        cout<<"Computation Time: "<<(end-start).toSeconds()*1000<<"ms"<<endl;

        cout<<"WBC bias torques: ";
        start=base::Time::now();
        robot_model.computeBiasForces();
        end=base::Time::now();
        cout<<robot_model.biasForces().transpose()<<endl;
        cout<<"Computation Time: "<<(end-start).toSeconds()*1000<<"ms"<<endl;

        for(int i = 0; i < model.dof_count; i++){
            BOOST_CHECK(fabs(robot_model.biasForces()[i] - C[i])  < 1e-9);
            for(int j = 0; j < model.dof_count; j++){
                BOOST_CHECK(fabs(robot_model.jointSpaceInertiaMatrix()(i,j) - H(i,j)) < 1e-9);
            }
        }
    }
}
