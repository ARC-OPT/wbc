#include <boost/test/unit_test.hpp>
#include "robot_models/KinematicRobotModelKDL.hpp"
#include "robot_models/KinematicChainKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

using namespace std;
using namespace wbc;

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

        CartesianState cstate;
        BOOST_CHECK_NO_THROW( cstate = robot_model.cartesianState("kuka_lbr_center", "kuka_lbr_l_tcp"));
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

        CartesianState cstate = robot_model.cartesianState("base", "ee");
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


    cout<<"\n----------------------- Testing Robot Model KDL ----------------------"<<endl<<endl;

    vector<string> joint_names;
    for(int i = 0; i < 7; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));

    srand(time(NULL));

    cout<<"Testing Model Creation .."<<endl<<endl;

    base::Pose object_pose;
    object_pose.position = base::Vector3d(0,0,2);
    object_pose.orientation.setIdentity();

    KinematicRobotModelKDL* robot_model = new KinematicRobotModelKDL();
    vector<RobotModelConfig> config(2);
    config[0].file = string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/kuka_lbr.urdf";
    config[1].file = string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/object.urdf";
    config[1].hook = "kuka_lbr_top_left_camera";
    config[1].initial_pose = object_pose;
    BOOST_CHECK_EQUAL(robot_model->configure(config, joint_names, "kuka_lbr_base"), true);

    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    joint_state.time = base::Time::now();
    for(base::JointState& j : joint_state.elements)
        j.position = rand();

    cout<<"Testing Model Update ...."<<endl<<endl;

    BOOST_CHECK_NO_THROW(robot_model->update(joint_state););

    cout<<"Testing FK ..."<<endl<<endl;

    KDL::JntArray joint_positions(joint_names.size());
    for(size_t i = 0; i < joint_names.size(); i++)
        joint_positions(i) = joint_state[i].position;

    KDL::Chain chain;
    BOOST_CHECK(robot_model->getTree().getChain("kuka_lbr_base", "kuka_lbr_l_tcp", chain) == true);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    KDL::Frame pose_kdl;
    fk_solver.JntToCart(joint_positions, pose_kdl);

    CartesianState state;
    BOOST_CHECK_NO_THROW(state = robot_model->cartesianState("kuka_lbr_base", "kuka_lbr_l_tcp"););

    for(int i = 0; i < 3; i++)
        BOOST_CHECK_EQUAL(pose_kdl.p(i), state.pose.position(i));

    double x,y,z,w;
    pose_kdl.M.GetQuaternion(x,y,z,w);

    BOOST_CHECK_EQUAL(x, state.pose.orientation.x());
    BOOST_CHECK_EQUAL(y, state.pose.orientation.y());
    BOOST_CHECK_EQUAL(z, state.pose.orientation.z());
    BOOST_CHECK_EQUAL(w, state.pose.orientation.w());

    cout<<"EE pose from KDL:"<<endl;
    cout<<"x: "<<pose_kdl.p(0)<<" y: "<<pose_kdl.p(1)<<" z: "<<pose_kdl.p(2)<<endl;
    cout<<"qx: "<<x<<" qy: "<<y<<" qz: "<<z<<" qw: "<<w<<endl<<endl;

    cout<<"EE pose from robot model:"<<endl;
    cout<<"x: "<<state.pose.position(0)<<" y: "<<state.pose.position(1)<<" z: "<<state.pose.position(2)<<endl;
    cout<<"qx: "<<state.pose.orientation.x()<<" qy: "<<state.pose.orientation.y()<<" qz: "<<state.pose.orientation.z()<<" qw: "<<state.pose.orientation.w()<<endl<<endl;


    cout<<"Testing Jacobian ..."<<endl<<endl;
    KDL::Jacobian jac_kdl(joint_names.size());
    KDL::ChainJntToJacSolver jac_solver(chain);
    jac_solver.JntToJac(joint_positions, jac_kdl);

    base::MatrixXd jac = robot_model->jacobian("kuka_lbr_base", "kuka_lbr_l_tcp");

    cout<<"Jacobian from KDL"<<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"Jacobian from model"<<endl;
    cout<<jac<<endl<<endl;

    for(int i = 0; i < jac.rows(); i++)
        for(int j = 0; j < jac.cols(); j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    cout<<"Object pose in camera coordinates: "<<endl;
    CartesianState st = robot_model->cartesianState("kuka_lbr_top_left_camera", "object");
    cout<<st.pose.position(0)<<" "<<st.pose.position(1)<<" "<<st.pose.position(2)<<endl;
    cout<<st.pose.orientation.x()<<" "<<st.pose.orientation.y()<<" "<<st.pose.orientation.z()<<" "<<st.pose.orientation.w()<<endl<<endl;

    cout<<"Object pose in base coordinates: "<<endl;
    st = robot_model->cartesianState("kuka_lbr_base", "object");
    cout<<st.pose.position(0)<<" "<<st.pose.position(1)<<" "<<st.pose.position(2)<<endl;
    cout<<st.pose.orientation.x()<<" "<<st.pose.orientation.y()<<" "<<st.pose.orientation.z()<<" "<<st.pose.orientation.w()<<endl<<endl;

    cout<<"Updating object pose by ...."<<endl<<endl;
    st.pose.position = base::Vector3d(0,1,0);
    st.pose.orientation.setIdentity();
    st.pose.orientation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::Unit(2));
    st.source_frame = "object";
    cout<<st.pose.position(0)<<" "<<st.pose.position(1)<<" "<<st.pose.position(2)<<endl;
    cout<<st.pose.orientation.x()<<" "<<st.pose.orientation.y()<<" "<<st.pose.orientation.z()<<" "<<st.pose.orientation.w()<<endl<<endl;

    vector<CartesianState> poses;
    poses.push_back(st);

    BOOST_CHECK_NO_THROW(robot_model->update(joint_state, poses));

    cout<<"Object pose in camera coordinates: "<<endl;
    st = robot_model->cartesianState("kuka_lbr_top_left_camera", "object");
    cout<<st.pose.position(0)<<" "<<st.pose.position(1)<<" "<<st.pose.position(2)<<endl;
    cout<<st.pose.orientation.x()<<" "<<st.pose.orientation.y()<<" "<<st.pose.orientation.z()<<" "<<st.pose.orientation.w()<<endl<<endl;

    cout<<"Object pose in base coordinates: "<<endl;
    st = robot_model->cartesianState("kuka_lbr_base", "object");
    cout<<st.pose.position(0)<<" "<<st.pose.position(1)<<" "<<st.pose.position(2)<<endl;
    cout<<st.pose.orientation.x()<<" "<<st.pose.orientation.y()<<" "<<st.pose.orientation.z()<<" "<<st.pose.orientation.w()<<endl<<endl;

    delete robot_model;
}
