#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <boost/test/unit_test.hpp>
#include <stdlib.h>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/Constraint.hpp"
#include "../src/WbcVelocity.hpp"
#include "../src/HierarchicalWDLSSolver.hpp"
#include "../src/GeneralizedInverse.hpp"
#include "../src/SolverTypes.hpp"
#include "../src/RobotModelKDL.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <string>

using namespace std;
using namespace wbc;


BOOST_AUTO_TEST_CASE(four_link)
{
    if(boost::unit_test::framework::master_test_suite().argc <= 1){
        std::cerr<<"Invalid number of command line args. Usage: ./test --run-test=robot_model_dynamic <file.urdf>"<<std::endl;
        return;
    }
    std::string urdf_file = boost::unit_test::framework::master_test_suite().argv[1];

    KDL::Tree tree;
    KDL::Chain chain;
    BOOST_CHECK(kdl_parser::treeFromFile(urdf_file, tree)  == true);
    BOOST_CHECK(tree.getChain("AH1", "PanToTilt", chain) == true);

    KDL::ChainFkSolverPos_recursive pos_solver(chain);
    KDL::ChainFkSolverVel_recursive vel_solver(chain);

    KDL::JntArray q(1);
    q(0) = -1.57;
    KDL::Frame pose;
    pos_solver.JntToCart(q, pose);

    double euler[3];
    pose.M.GetEulerZYX(euler[2], euler[1], euler[0]);
    std::cout<<euler[0]<<" "<<euler[1]<<" "<<euler[2]<<std::endl;

    KDL::JntArrayVel q_dot(1);
    KDL::FrameVel frame_vel;
    q_dot.q(0) = 0.0;
    q_dot.qdot(0) = 0.5;
    vel_solver.JntToCart(q_dot, frame_vel);

    std::cout<<frame_vel.deriv()<<std::endl;

}

BOOST_AUTO_TEST_CASE(derived)
{
    class A{
    public:
        A(){
            std::cout<<"Calling constructor of A"<<std::endl;
        }
        virtual ~A(){
            std::cout<<"Calling destructor of A"<<std::endl;
        }
        virtual void testFunc(){
            std::cout<<"Calling testFunc of A"<<std::endl;
        }
    };

    class Derived : public A{
    public:
        Derived(){
            std::cout<<"Calling constructor of Derived"<<std::endl;
        }
        virtual ~Derived(){
            std::cout<<"Calling destructor of Derived"<<std::endl;
        }
        virtual void testFunc(){
            std::cout<<"Calling testFunc of Derived"<<std::endl;
        }
    };

    std::cout<<"Constructing Derived: "<<std::endl;
    A* derived = new Derived();

    std::cout<<"Calling testFunc(): "<<std::endl;
    derived->testFunc();

    std::cout<<"detroying derived: "<<std::endl;
    delete derived;
}

/**
 * Test hierarchical solver with random input data
 */

BOOST_AUTO_TEST_CASE(solver)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 2;
    const double NORM_MAX = 5.75;

    HierarchicalWDLSSolver solver;
    std::vector<int> ny_per_prio(1,NO_CONSTRAINTS);
    BOOST_CHECK(solver.configure(ny_per_prio, NO_JOINTS) == true);
    solver.setNormMax(NORM_MAX);

    std::vector<LinearEqnSystem> input(1);
    input[0].resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        input[0].A.data()[i] = (rand()%1000)/1000.0;

    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        input[0].y_ref.data()[i] = (rand()%1000)/1000.0;

    input[0].W_row.setConstant(1);
    input[0].W_col.setConstant(1);

    cout<<"............Testing Hierarchical Solver "<<endl<<endl;
    cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
    cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;
    cout<<"\n----------------------- Solver Input ----------------------"<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"Priority: "<<i<<endl;
        cout<<"A: "<<endl; cout<<input[i].A<<endl;
        cout<<"y_ref: "<<endl; cout<<input[i].y_ref<<endl;
        cout<<endl;
    }

    Eigen::VectorXd solver_output;
    try{
        solver.solve(input,  solver_output);
    }
    catch(std::exception e){
        BOOST_ERROR("Solver.solve threw an exception");
    }

    cout<<"----------------- Solver Output: ------------------------"<<endl;
    cout<<"q_ref = "<<endl;
    cout<<solver_output<<endl;
    cout<<"\nTest: "<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"----------------- Priority: "<<i<<" ------------------"<<endl;
        Eigen::VectorXd test = input[i].A*solver_output;
        cout<<"A * q_ref: "<<endl;
        cout<<test<<endl; cout<<endl;
        for(uint j = 0; j < NO_CONSTRAINTS; j++)
            BOOST_CHECK(fabs(test(j) - input[i].y_ref(j)) < 1e-9);
    }

    cout<<"\n............................."<<endl;
}

BOOST_AUTO_TEST_CASE(pseudo_inverse)
{
    srand (time(NULL));
    const uint N_ROWS = 3;
    const uint N_COLS = 5;

    GeneralizedInverse inv(N_ROWS, N_COLS);

    Eigen::MatrixXd in(N_ROWS, N_COLS);
    Eigen::MatrixXd out(N_COLS, N_ROWS);

    for(uint i = 0; i < N_ROWS*N_COLS; i++ )
        in.data()[i] = (rand()%1000)/1000.0;

    inv.computeInverse(in, out);

    Eigen::MatrixXd res(N_ROWS,N_ROWS);
    res = in * out;

    std::cout << "--------- Input Mat --------" << std::endl << std::endl;
    std::cout << in << std::endl << std::endl;

    std::cout << "Weighting time: " << inv.time_weighting_ << " seconds " << std::endl << std::endl;
    std::cout << "SVD time: " << inv.time_svd_ << " seconds " << std::endl << std::endl;
    std::cout << "Multiplication time: " << inv.time_multiplying_ << " seconds " << std::endl << std::endl;
    std::cout << "Total Computation time: " << inv.time_total_ << " seconds " << std::endl << std::endl;

    std::cout << "--------- Output Mat --------" << std::endl << std::endl;
    std::cout<< out << std::endl << std::endl;

    std::cout << " ----- Input * Output Mat: ----- " << std::endl << std::endl;
    std::cout<< res << std::endl;

    for(uint i = 0; i < N_ROWS; i++)
    {
        for(uint j = 0; j < N_ROWS; j++)
        {
            if(i == j)
                BOOST_CHECK(fabs(res(i,j) - 1 )  < 1e-5);
            else
                BOOST_CHECK(fabs(res(i,j))  < 1e-5);
        }
    }
}

BOOST_AUTO_TEST_CASE(damped_pseudo_inverse)
{
    srand (time(NULL));
    const uint N_ROWS = 3;
    const uint N_COLS = 5;

    GeneralizedInverse inv(N_ROWS, N_COLS);
    inv.setConstantDamping(0.01);

    Eigen::MatrixXd in(N_ROWS, N_COLS);
    Eigen::MatrixXd out(N_COLS, N_ROWS);

    for(uint i = 0; i < N_ROWS*N_COLS; i++ )
        in.data()[i] = (rand()%1000)/1000.0;

    inv.computeInverse(in, out);

    Eigen::MatrixXd res(N_ROWS,N_ROWS);
    res = in * out;

    std::cout << "--------- Input Mat --------" << std::endl << std::endl;
    std::cout << in << std::endl << std::endl;

    std::cout << "Weighting time: " << inv.time_weighting_ << " seconds " << std::endl << std::endl;
    std::cout << "SVD time: " << inv.time_svd_ << " seconds " << std::endl << std::endl;
    std::cout << "Multiplication time: " << inv.time_multiplying_ << " seconds " << std::endl << std::endl;
    std::cout << "Total Computation time: " << inv.time_total_ << " seconds " << std::endl << std::endl;
    std::cout << " Current Damping: " << inv.damping_ << std::endl << std::endl;

    std::cout << "--------- Output Mat --------" << std::endl << std::endl;
    std::cout<< out << std::endl << std::endl;

    std::cout << " ----- Input * Output Mat: ----- " << std::endl << std::endl;
    std::cout<< res << std::endl;

    for(uint i = 0; i < N_ROWS; i++)
    {
        for(uint j = 0; j < N_ROWS; j++)
        {
            if(i == j)
                BOOST_CHECK(fabs(res(i,j) - 1 )  < 0.01);
            else
                BOOST_CHECK(fabs(res(i,j))  < 0.01);
        }
    }
}

BOOST_AUTO_TEST_CASE(auto_damped_pseudo_inverse)
{
    srand (time(NULL));
    const uint N_ROWS = 3;
    const uint N_COLS = 5;
    const double NORM_MAX = 2.0;

    GeneralizedInverse inv(N_ROWS, N_COLS);
    inv.setNormMaxDamping(NORM_MAX);

    Eigen::MatrixXd in(N_ROWS, N_COLS);
    Eigen::MatrixXd out(N_COLS, N_ROWS);

    for(uint i = 0; i < N_ROWS*N_COLS; i++ )
        in.data()[i] = (rand()%1000)/1000.0;

    inv.computeInverse(in, out);

    Eigen::MatrixXd res(N_ROWS,N_ROWS);
    res = in * out;

    std::cout << "--------- Input Mat --------" << std::endl << std::endl;
    std::cout << in << std::endl << std::endl;

    std::cout << "Norm Max: " << NORM_MAX << std::endl << std::endl;
    std::cout << "1 /Norm Max: " << 1/inv.norm_max_ << std::endl << std::endl;
    std::cout << "Current Damping: " << inv.damping_ << std::endl << std::endl;

    std::cout << "--------- Singular Values --------" << std::endl << std::endl;
    std::cout << inv.singular_vals_ << std::endl << std::endl;

    std::cout << "Weighting time: " << inv.time_weighting_ << " seconds " << std::endl << std::endl;
    std::cout << "SVD time: " << inv.time_svd_ << " seconds " << std::endl << std::endl;
    std::cout << "Multiplication time: " << inv.time_multiplying_ << " seconds " << std::endl << std::endl;
    std::cout << "Total Computation time: " << inv.time_total_ << " seconds " << std::endl << std::endl;

    std::cout << "--------- Output Mat --------" << std::endl << std::endl;
    std::cout<< out << std::endl << std::endl;

    std::cout << " ----- Input * Output Mat: ----- " << std::endl << std::endl;
    std::cout<< res << std::endl;

    for(uint i = 0; i < N_ROWS; i++)
    {
        for(uint j = 0; j < N_ROWS; j++)
        {
            if(i == j)
                BOOST_CHECK(fabs(res(i,j) - 1 )  < 0.01);
            else
                BOOST_CHECK(fabs(res(i,j))  < 0.01);
        }
    }
}

BOOST_AUTO_TEST_CASE(weighted_pseudo_inverse)
{
    srand (time(NULL));
    const uint N_ROWS = 3;
    const uint N_COLS = 5;
    const uint COL_NUMBER_WITH_ZERO_WEIGHT = 3;
    const uint ROW_NUMBER_WITH_ZERO_WEIGHT = 1;

    GeneralizedInverse inv(N_ROWS, N_COLS);

    Eigen::MatrixXd in(N_ROWS, N_COLS);
    Eigen::MatrixXd out(N_COLS, N_ROWS);

    Eigen::VectorXd col_weights, row_weights;
    col_weights.setOnes(N_COLS);
    row_weights.setOnes(N_ROWS);

    col_weights(COL_NUMBER_WITH_ZERO_WEIGHT) = 0;
    row_weights(ROW_NUMBER_WITH_ZERO_WEIGHT) = 0;

    inv.setColWeights(col_weights);
    inv.setRowWeights(row_weights);

    for(uint i = 0; i < N_ROWS*N_COLS; i++ )
        in.data()[i] = (rand()%1000)/1000.0;

    inv.computeInverse(in, out);

    Eigen::MatrixXd res(N_ROWS,N_ROWS);
    res = in * out;

    std::cout << "--------- Input Mat --------" << std::endl << std::endl;
    std::cout << in << std::endl << std::endl;

    std::cout << "---------- Column weights -------- " << std::endl << std::endl;
    std::cout << inv.col_weights_ << std::endl << std::endl;

    std::cout << "---------- Row weights -------- " << std::endl << std::endl;
    std::cout << inv.row_weights_ << std::endl << std::endl;

    std::cout << "--------- Weighted input Mat --------" << std::endl << std::endl;
    std::cout << inv.weighted_mat_ << std::endl << std::endl;

    std::cout << "--------- Output Mat --------" << std::endl << std::endl;
    std::cout<< out << std::endl << std::endl;

    std::cout << "Weighting time: " << inv.time_weighting_ << " seconds " << std::endl << std::endl;
    std::cout << "SVD time: " << inv.time_svd_ << " seconds " << std::endl << std::endl;
    std::cout << "Multiplication time: " << inv.time_multiplying_ << " seconds " << std::endl << std::endl;
    std::cout << "Total Computation time: " << inv.time_total_ << " seconds " << std::endl << std::endl;

    std::cout << " ----- Input * Output Mat: ----- " << std::endl << std::endl;
    std::cout<< res << std::endl;

    for(uint i = 0; i < N_ROWS; i++)
    {
        for(uint j = 0; j < N_ROWS; j++)
        {
            if( i == ROW_NUMBER_WITH_ZERO_WEIGHT)
                continue;

            if(i == j)
                BOOST_CHECK(fabs(res(i,j) - 1 )  < 1e-5);
            else
                BOOST_CHECK(fabs(res(i,j))  < 1e-5);
        }
    }
}

BOOST_AUTO_TEST_CASE(robot_model_dynamic)
{

    srand (time(NULL));

    if(boost::unit_test::framework::master_test_suite().argc <= 1){
        std::cerr<<"Invalid number of command line args. Usage: ./test --run-test=robot_model_dynamic <file.urdf>"<<std::endl;
        return;
    }
    std::string urdf_file = boost::unit_test::framework::master_test_suite().argv[1];

    KDL::Tree full_tree;
    KDL::Chain chain;
    BOOST_CHECK(kdl_parser::treeFromFile(urdf_file, full_tree)  == true);
    KDL::Tree tree;
    BOOST_CHECK(full_tree.getChain("Chest", "Hand_l", chain) == true);

    tree.addSegment(KDL::Segment("Chest", KDL::Joint("Chest",KDL::Joint::None),KDL::Frame::Identity()), "root");
    tree.addChain(chain, "Chest");

    Eigen::Vector3d grav(0,0,-9.81);
    RobotModelKDLDyn model(tree,grav);
    BOOST_CHECK(model.addTaskFrame("Hand_l") == true);

    base::samples::Joints joint_state;
    TaskFrameKDL* tf = model.getTaskFrame("Hand_l");
    BOOST_CHECK(tf != 0);

    joint_state.resize(tf->joint_names_.size());
    joint_state.names = tf->joint_names_;

    for(uint i = 0; i < joint_state.size(); i++){
        joint_state[i].position = 0;
        joint_state[i].speed = 0;
    }
    joint_state[0].position = M_PI/2;
    model.update(joint_state);

    std::cout<<"Jacobian: "<<tf->jac_.data<<std::endl<<std::endl;
    std::cout<<"Pose: "<<tf->pose_<<std::endl<<std::endl;
    std::cout<<"Gravity: "<<tf->jnt_gravity_<<std::endl<<std::endl;
    std::cout<<"Inertia: "<<tf->jnt_inertia_<<std::endl<<std::endl;
    std::cout<<"Coriolis: "<<tf->jnt_coriolis_<<std::endl<<std::endl;

    //Check if inertia mat is positive definite
    Eigen::MatrixXd U, V;
    Eigen::VectorXd S, tmp;
    U.resize(tf->jnt_inertia_.rows(), tf->jnt_inertia_.cols());
    V.resize(tf->jnt_inertia_.cols(), tf->jnt_inertia_.cols());
    S.resize(tf->jnt_inertia_.cols());
    tmp.resize(tf->jnt_inertia_.cols());
    KDL::svd_eigen_HH(tf->jnt_inertia_, U, S, V, tmp);

    for(uint i = 0; i < S.size(); i++)
        BOOST_CHECK(S(i) > 0);
}
