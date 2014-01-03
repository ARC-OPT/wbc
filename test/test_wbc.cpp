#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <boost/test/unit_test.hpp> 
#include "../src/HierarchicalWDLSSolver.hpp"
#include <stdlib.h>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/Wbc.hpp"

using namespace std;


BOOST_AUTO_TEST_CASE(test_solver)
{
   srand (time(NULL));

   const uint NO_JOINTS = 7;
   const uint NO_CONSTRAINTS = 2;

   HierarchicalWDLSSolver solver;
   std::vector<unsigned int> ny_per_prio(1,NO_CONSTRAINTS);
   solver.configure(ny_per_prio, NO_JOINTS);
   solver.setNormMax(5.75);

   Eigen::VectorXd joint_weights;
   joint_weights.resize(NO_JOINTS);
   joint_weights.setConstant(1);
   joint_weights(0) = 1;
   solver.setJointWeights(joint_weights);

   Eigen::VectorXd task_weights;
   task_weights.resize(NO_CONSTRAINTS);
   task_weights.setConstant(1);
   task_weights(0) = 1.0;
   task_weights(1) = 1.0;
   solver.setTaskWeights(task_weights, 0);


   Eigen::MatrixXd A(NO_CONSTRAINTS,NO_JOINTS);
   for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ ) A.data()[i] = (rand()%1000)/1000.0;
   Eigen::VectorXd y(NO_CONSTRAINTS);
   y.setConstant(1);

   std::vector<Eigen::MatrixXd> A_prio;
   A_prio.push_back(A);
   for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ ) A.data()[i] = (rand()%1000)/1000.0;
   //A_prio.push_back(A);

   std::vector<Eigen::VectorXd> y_prio;
   for(uint i = 0; i < NO_CONSTRAINTS; i++ ) y.data()[i] = (rand()%1000)/1000.0;
   y_prio.push_back(y);
   for(uint i = 0; i < NO_CONSTRAINTS; i++ ) y.data()[i] = (rand()%1000)/1000.0;
   //y_prio.push_back(y);

   cout<<"............Testing Hierarchical Solver "<<endl<<endl;
   cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
   cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
   cout<<"No of joints: "<<NO_JOINTS<<endl;
   cout<<"Solver Input: "<<endl;
   for(uint i = 0; i < ny_per_prio.size(); i++){
       cout<<"Priority: "<<i<<endl;
       cout<<"A: "<<endl; cout<<A_prio[i]<<endl;
       cout<<"y: "<<endl; cout<<y_prio[i]<<endl;
       cout<<endl;
   }

   Eigen::VectorXd solver_output;
   solver.solve(A_prio, y_prio, solver_output);

   cout<<"Output: "<<solver_output<<endl;
   cout<<"Test: "<<endl;
   for(uint i = 0; i < ny_per_prio.size(); i++){
       cout<<"Priority: "<<i<<endl;
       Eigen::VectorXd test = A_prio[i]*solver_output;
       cout<<"A*q: "<<test<<endl; cout<<endl;
       for(uint j = 0; j < NO_CONSTRAINTS; j++)
           BOOST_CHECK_EQUAL(fabs(test(j) - y_prio[i](j)) < 1e-9, true);
   }

   cout<<endl;
}


BOOST_AUTO_TEST_CASE(test_wbc)
{
    int argc = boost::unit_test::framework::master_test_suite().argc;
    char **argv = boost::unit_test::framework::master_test_suite().argv;

    argc-=2;
    argv+=2;

    std::string urdf_file = argv[0];

    KDL::Tree tree, reduced_tree;
    BOOST_CHECK_EQUAL(kdl_parser::treeFromFile(urdf_file, tree), true);
    KDL::Chain right_hand_chain, left_hand_chain, chest_chain;
    tree.getChain("Chest", "Hand_l", chest_chain);
    tree.getChain("Rover_base", "Hand_l", left_hand_chain);
    tree.getChain("Rover_base", "Hand_r", right_hand_chain);
    reduced_tree.addSegment(KDL::Segment("Rover_base", KDL::Joint("Rover_base",KDL::Joint::None),KDL::Frame::Identity()), "root");
    reduced_tree.addChain(right_hand_chain, "Rover_base");
    reduced_tree.addChain(chest_chain, "Chest");

    Wbc wbc(reduced_tree, WBC_TYPE_VELOCITY);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController_r", 0, "Rover_base", "Hand_r", 6), true);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController_l", 0, "Rover_base", "Hand_l", 6), true);
    BOOST_CHECK_EQUAL(wbc.configure(), true);

    base::samples::Joints status;
    base::commands::Joints command;

    status.resize(chest_chain.getNrOfJoints() + right_hand_chain.getNrOfJoints());
    uint idx = 0;
    for(uint i = 0; i < right_hand_chain.getNrOfSegments(); i++){
        KDL::Segment seg = right_hand_chain.getSegment(i);
        if(seg.getJoint().getType() != KDL::Joint::None)
            status.names[idx++] = seg.getJoint().getName();
    }
    for(uint i = 0; i < chest_chain.getNrOfSegments(); i++){
        KDL::Segment seg = chest_chain.getSegment(i);
        if(seg.getJoint().getType() != KDL::Joint::None)
            status.names[idx++] = seg.getJoint().getName();
    }
    for(uint i = 0; i < status.size(); i++){
        status[i].position = 0.3;
        status[i].speed = status[i].effort = 0;
    }


    WbcInputMap input;
    Eigen::VectorXd cart_in_r(6);
    Eigen::VectorXd cart_in_l(6);
    cart_in_r.setZero();
    cart_in_l.setZero();
    cart_in_r(0) = 0.1;
    cart_in_l(2) = 0.1;
    input["CartesianPoseController_r"] = cart_in_r;
    input["CartesianPoseController_l"] = cart_in_l;
    wbc.solve(input, status, command);

    //Test
    KDL::ChainFkSolverVel_recursive solver_r(right_hand_chain);
    KDL::ChainFkSolverVel_recursive solver_l(left_hand_chain);
    KDL::FrameVel p_out_r, p_out_l;
    KDL::JntArrayVel q_dot_in_r(right_hand_chain.getNrOfJoints());
    KDL::JntArrayVel q_dot_in_l(left_hand_chain.getNrOfJoints());

    for(uint i = 0; i < right_hand_chain.getNrOfJoints(); i++){
        q_dot_in_r.q(i) = 0.3;
        q_dot_in_r.qdot(i) = command[i].speed;
    }

    for(uint i = 0; i < 4; i++){
        q_dot_in_l.q(i) = 0.3;
        q_dot_in_l.qdot(i) = command[i].speed;
    }
    for(uint i = 0; i < chest_chain.getNrOfJoints(); i++){
        q_dot_in_l.q(i + 4) = 0.3;
        q_dot_in_l.qdot(i + 4) = command[i + right_hand_chain.getNrOfJoints()].speed;
    }
    solver_r.JntToCart(q_dot_in_r, p_out_r);
    solver_l.JntToCart(q_dot_in_l, p_out_l);

    cout<<"Desired Cart Velocity right: "<<endl;
    cout<<cart_in_r.transpose()<<endl<<endl;

    cout<<"Desired Cart Velocity left: "<<endl;
    cout<<cart_in_l.transpose()<<endl<<endl;

    cout<<"Actual Cart velocity right: "<<endl;
    cout<<p_out_r.deriv()<<endl<<endl;

    cout<<"Actual Cart Velocity left: "<<endl;
    cout<<p_out_l.deriv()<<endl<<endl;

    for(uint i = 0; i < 6; i++ ){
        BOOST_CHECK_EQUAL(fabs(p_out_r.deriv()(i) - cart_in_r(i)) < 1e-5, true);
        BOOST_CHECK_EQUAL(fabs(p_out_l.deriv()(i) - cart_in_l(i)) < 1e-5, true);
    }
}

BOOST_AUTO_TEST_CASE(test_wbc_invalid_sub_task)
{
    int argc = boost::unit_test::framework::master_test_suite().argc;
    char **argv = boost::unit_test::framework::master_test_suite().argv;

    argc-=2;
    argv+=2;

    std::string urdf_file = argv[0];

    KDL::Tree tree;
    BOOST_CHECK_EQUAL(kdl_parser::treeFromFile(urdf_file, tree), true);

    Wbc wbc(tree, WBC_TYPE_VELOCITY);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController", 0, "Rover_base", "Hand_r", 6), true);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController", 0, "Rover_base", "Hand_l", 6), true);


    BOOST_CHECK_EQUAL(wbc.addSubTask("VSController", 5, "chest", "Hand_r", 6), true);

    BOOST_CHECK_EQUAL(wbc.addSubTask("VSController", 5, "Chest", "Chest", 6), true);

    BOOST_CHECK_EQUAL(wbc.configure(), true);
}
