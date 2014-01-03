#include <boost/test/unit_test.hpp> 
#include "../src/HierarchicalWDLSSolver.hpp"
#include <stdlib.h>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/Wbc.hpp""

using namespace std;


BOOST_AUTO_TEST_CASE(test_solver)
{
   srand (time(NULL));

   const int NO_JOINTS = 7;
   const int NO_CONSTRAINTS = 2;

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
   for(int i = 0; i < NO_CONSTRAINTS; i++ ) y.data()[i] = (rand()%1000)/1000.0;
   y_prio.push_back(y);
   for(int i = 0; i < NO_CONSTRAINTS; i++ ) y.data()[i] = (rand()%1000)/1000.0;
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

    KDL::Tree tree;
    BOOST_CHECK_EQUAL(kdl_parser::treeFromFile(urdf_file, tree), true);

    Wbc wbc(tree, WBC_TYPE_VELOCITY);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController_r", 0, "Rover_base", "Hand_r", 6), true);
    BOOST_CHECK_EQUAL(wbc.addSubTask("CartesianPoseController_l", 0, "Rover_base", "Hand_l", 6), true);
    BOOST_CHECK_EQUAL(wbc.addSubTask("VSController", 5, "Chest", "Hand_r", 6), true);


    BOOST_CHECK_EQUAL(wbc.configure(), true);
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
