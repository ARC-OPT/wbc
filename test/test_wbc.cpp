#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <boost/test/unit_test.hpp>
#include <stdlib.h>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/WbcVelocity.hpp"
#include "../src/RobotModel.hpp"
#include "../src/HierarchicalWDLSSolver.hpp"

using namespace std;
using namespace wbc;


BOOST_AUTO_TEST_CASE(test_solver)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 4;
    const uint NO_RUNS = 1;

    HierarchicalWDLSSolver solver;
    std::vector<uint> ny_per_prio(1,NO_CONSTRAINTS);
    solver.configure(ny_per_prio, NO_JOINTS);
    solver.setNormMax(5.75);

    //Set joint weights
    Eigen::VectorXd joint_weights;
    joint_weights.resize(NO_JOINTS);
    joint_weights.setConstant(1);
    solver.setJointWeights(joint_weights);

    //Set task weights
    Eigen::VectorXd task_weights;
    task_weights.resize(NO_CONSTRAINTS);
    task_weights.setConstant(1);
    task_weights(0) = 0.0;
    task_weights(1) = 1.0;
    solver.setTaskWeights(task_weights, 0);

    cout<<"............Testing Hierarchical Solver ............ "<<endl<<endl;
    for(uint run_no = 1; run_no <= NO_RUNS; run_no++){
        cout<<"Run No: "<<run_no<<endl<<endl;
        Eigen::MatrixXd A(NO_CONSTRAINTS,NO_JOINTS);
        for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ ) A.data()[i] = (rand()%1000)/1000.0;
        Eigen::VectorXd y(NO_CONSTRAINTS);
        y.setConstant(1);


        std::vector<Eigen::MatrixXd> A_prio;
        A_prio.push_back(A);

        std::vector<Eigen::VectorXd> y_prio;
        for(uint i = 0; i < NO_CONSTRAINTS; i++ ) y.data()[i] = (rand()%1000)/1000.0;
        y_prio.push_back(y);

        cout<<"............Testing Hierarchical Solver "<<endl<<endl;
        cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
        cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
        cout<<"No of joints: "<<NO_JOINTS<<endl;
        cout<<"\nSolver Input: "<<endl;
        for(uint i = 0; i < ny_per_prio.size(); i++){
            cout<<"Priority: "<<i<<endl;
            cout<<"A: "<<endl; cout<<A_prio[i]<<endl;
            cout<<"y: "<<endl; cout<<y_prio[i]<<endl;
            cout<<endl;
        }

        Eigen::VectorXd solver_output;
        solver.solve(A_prio, y_prio, solver_output);

        cout<<"Solver damping: "<<solver.getCurDamping()<<endl;
        cout<<"Solver Output: "<<solver_output<<endl;
        cout<<"\nTest: "<<endl;
        for(uint i = 0; i < ny_per_prio.size(); i++){
            cout<<"Priority: "<<i<<endl;
            Eigen::VectorXd test = A_prio[i]*solver_output;
            cout<<"A*q: "<<test<<endl; cout<<endl;
            for(uint j = 0; j < NO_CONSTRAINTS; j++)
                BOOST_CHECK_EQUAL(fabs(test(j) - y_prio[i](j)) < 1e-9, true);
        }

        cout<<"\n............................."<<endl;
    }
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
    KDL::Chain right_hand_chain;
    tree.getChain("Rover_base", "Hand_r", right_hand_chain);
    reduced_tree.addSegment(KDL::Segment("Rover_base", KDL::Joint("Rover_base",KDL::Joint::None),KDL::Frame::Identity()), "root");
    reduced_tree.addChain(right_hand_chain, "Rover_base");

    WbcVelocity wbc;
    std::vector<SubTaskConfig> config;
    SubTaskConfig conf;
    conf.root = "Rover_base";
    conf.tip = "Hand_r";
    conf.type = task_type_cartesian;
    config.push_back(conf);
    RobotModel robot;
    //BOOST_CHECK_EQUAL(robot.configure(tree, config), true);
    //BOOST_CHECK_EQUAL(wbc.configure(config, robot.no_of_joints_), true);

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

    //TODO
}
