#include <boost/test/unit_test.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include "robot_models/KinematicRobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(robot_model_kdl){


    cout<<"\n----------------------- Testing Robot Model KDL ----------------------"<<endl<<endl;

    vector<string> joint_names;
    for(int i = 0; i < 7; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));

    srand(time(NULL));

    cout<<"Testing Model Creation .."<<endl<<endl;

    base::samples::RigidBodyState object_pose;
    object_pose.position = base::Vector3d(0,0,2);
    object_pose.orientation.setIdentity();

    KinematicRobotModelKDL* robot_model = new KinematicRobotModelKDL();
    vector<RobotModelConfig> config(2);
    config[0].file = std::string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/kuka_lbr.urdf";
    config[1].file = std::string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/object.urdf";
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

    base::samples::RigidBodyState rbs;
    BOOST_CHECK_NO_THROW(rbs = robot_model->rigidBodyState("kuka_lbr_base", "kuka_lbr_l_tcp"););

    base::samples::RigidBodyState rbs_converted;
    kdl_conversions::KDL2RigidBodyState(pose_kdl, rbs_converted);

    for(int i = 0; i < 3; i++)
        BOOST_CHECK_EQUAL(rbs.position(i), rbs_converted.position(i));

    BOOST_CHECK_EQUAL(rbs.orientation.x(), rbs_converted.orientation.x());
    BOOST_CHECK_EQUAL(rbs.orientation.y(), rbs_converted.orientation.y());
    BOOST_CHECK_EQUAL(rbs.orientation.z(), rbs_converted.orientation.z());
    BOOST_CHECK_EQUAL(rbs.orientation.w(), rbs_converted.orientation.w());

    cout<<"EE pose from KDL:"<<endl;
    cout<<"x: "<<rbs_converted.position(0)<<" y: "<<rbs_converted.position(1)<<" z: "<<rbs_converted.position(2)<<endl;
    cout<<"qx: "<<rbs_converted.orientation.x()<<" qy: "<<rbs_converted.orientation.y()<<" qz: "<<rbs_converted.orientation.z()<<" qw: "<<rbs_converted.orientation.w()<<endl<<endl;

    cout<<"EE pose from robot model:"<<endl;
    cout<<"x: "<<rbs.position(0)<<" y: "<<rbs.position(1)<<" z: "<<rbs.position(2)<<endl;
    cout<<"qx: "<<rbs.orientation.x()<<" qy: "<<rbs.orientation.y()<<" qz: "<<rbs.orientation.z()<<" qw: "<<rbs.orientation.w()<<endl<<endl;


    cout<<"Testing Jacobian ..."<<endl<<endl;
    KDL::Jacobian jac_kdl(joint_names.size());
    KDL::ChainJntToJacSolver jac_solver(chain);
    jac_solver.JntToJac(joint_positions, jac_kdl);
    jac_kdl.changeRefPoint(-pose_kdl.p);

    base::MatrixXd jac = robot_model->jacobian("kuka_lbr_base", "kuka_lbr_l_tcp");

    cout<<"Jacobian from KDL"<<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"Jacobian from model"<<endl;
    cout<<jac<<endl<<endl;

    for(int i = 0; i < jac.rows(); i++)
        for(int j = 0; j < jac.cols(); j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    delete robot_model;
}
