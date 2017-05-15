#include <kdl/frames_io.hpp>
#include <boost/test/unit_test.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/WbcVelocity.hpp"
#include "../src/HierarchicalLeastSquaresSolver.hpp"
#include "../src/KinematicRobotModelKDL.hpp"
#include "../src/OptProblem.hpp"
#include "../src/RobotModelConfig.hpp"
#include "../src/TaskFrame.hpp"

using namespace std;
using namespace wbc;

/**
 * Test hierarchical solver with random input data
 */

BOOST_AUTO_TEST_CASE(hierarchical_ls_solver)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 2;
    const double NORM_MAX = 5.75;
    const double MIN_EIGENVALUE = 1e-9;

    HierarchicalLeastSquaresSolver solver;
    std::vector<int> ny_per_prio(1,NO_CONSTRAINTS);

    BOOST_CHECK(solver.configure(ny_per_prio, NO_JOINTS) == true);

    BOOST_NOEXCEPT(solver.setMaxSolverOutputNorm(NORM_MAX));
    BOOST_NOEXCEPT(solver.setMinEigenvalue(MIN_EIGENVALUE));

    HierarchicalWeightedLS input;
    WeightedLS prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        prio_0.A.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.y_ref.data()[i] = (rand()%1000)/1000.0;
    prio_0.W.setConstant(1);

    input.prios.push_back(prio_0);

    cout<<"\n----------------------- Testing Hierarchical Solver ----------------------"<<endl<<endl;
    cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
    cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;
    cout<<"\n----------------------- Solver Input ----------------------"<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"Priority: "<<i<<endl;
        cout<<"Constraint Matrix: "<<endl; cout<<prio_0.A<<endl;
        cout<<"Reference: "<<endl; cout<<prio_0.y_ref<<endl;
        cout<<endl;
    }

    Eigen::VectorXd solver_output;
    try{
        solver.solve(input, solver_output);
    }
    catch(std::exception e){
        BOOST_ERROR("Solver.solve threw an exception");
    }

    cout<<"---------------------- Solver Output ------------------------"<<endl;
    cout<<"q_dot = "<<endl;
    cout<<solver_output<<endl;
    cout<<"---------------------- TEST ------------------------"<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<" ---- Priority: "<<i<<" ----"<<endl<<endl;
        Eigen::VectorXd test = prio_0.A*solver_output;
        cout<<"A * q_dot: "<<endl;
        cout<<test<<endl; cout<<endl;
        for(uint j = 0; j < NO_CONSTRAINTS; j++)
            BOOST_CHECK(fabs(test(j) - prio_0.y_ref(j)) < 1e-9);
    }

    cout<<"\n............................."<<endl;
}

BOOST_AUTO_TEST_CASE(wbc_velocity){

    WbcVelocity wbc;
    KinematicRobotModelKDL robot_model;
    HierarchicalLeastSquaresSolver solver;

    ///// Create WBC config
    ///

    std::vector<ConstraintConfig> wbc_config;

    // Constraint for Cartesian Position Control
    ConstraintConfig cart_constraint;
    cart_constraint.name       = "cart_pos_ctrl";
    cart_constraint.type       = cart;
    cart_constraint.priority   = 0; // highest
    cart_constraint.root       = "kuka_lbr_base";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_base";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    ///// Configure robot model
    ///

    RobotModelConfig robot_model_cfg("../../data/kuka_lbr.urdf");
    std::vector<RobotModelConfig> models;
    models.push_back(robot_model_cfg);

    std::vector<std::string> task_frames;
    task_frames.push_back("kuka_lbr_center");
    task_frames.push_back("kuka_lbr_top_left_camera");

    BOOST_CHECK(robot_model.configure(models, wbc.getTaskFrameIDs(wbc_config), "kuka_lbr_base", std::vector<std::string>()) == true);

    cout<<"Configured robot model ..."<<endl;

    ///// Configure WBC
    ///

    BOOST_CHECK(wbc.configure(wbc_config, robot_model.getJointNames()) == true);

    cout<<"Configured wbc..."<<endl;

    ///// Configure Solver
    ///

    BOOST_CHECK(solver.configure(wbc.getConstraintVariablesPerPrio(), robot_model.getJointNames().size()) == true);

    cout<<"Configured solver..."<<endl;

    ///// Run Control loop
    ///

    base::samples::Joints cur_joint_pos;
    cur_joint_pos.resize(7);
    for(int i = 0; i < 7; i++){
        cur_joint_pos[i].position = 1.5;
        stringstream ss;
        ss << i+1;
        cur_joint_pos.names[i] = "kuka_lbr_l_joint_" + ss.str();
    }

    BOOST_NOEXCEPT(robot_model.update(cur_joint_pos));
    TaskFrame* tf = robot_model.getTaskFrame(cart_constraint.tip);
    BOOST_CHECK(tf);
    base::samples::RigidBodyState target_pose, cur_pose;
    target_pose.position = tf->pose.position;
    target_pose.orientation.setIdentity();

    for(int i = 0; i < 7; i++)
        cur_joint_pos[i].position = 0.1;

    base::VectorXd solver_output(7);
    BOOST_NOEXCEPT(robot_model.update(cur_joint_pos));

    base::VectorXd weights(6);
    weights.setOnes();
    weights.segment(3,3).setZero(); // Set orientation task weights to zero
    BOOST_NOEXCEPT(wbc.setConstraintWeights(cart_constraint.name, weights));

    HierarchicalWeightedLS opt_problem;
    double cycle_time = 0.1;
    bool reached = false;
    while(!reached){

        robot_model.update(cur_joint_pos);
        robot_model.getState(cart_constraint.ref_frame, cart_constraint.tip, cur_pose);

        printf("Target Pos: %.4f %.4f %.4f\n", target_pose.position(0), target_pose.position(1), target_pose.position(2));
        printf("   Cur Pos: %.4f %.4f %.4f\n\n", cur_pose.position(0), cur_pose.position(1), cur_pose.position(2));

        base::samples::RigidBodyState ref;
        ref.velocity = target_pose.position - cur_pose.position;
        ref.angular_velocity.setZero();

        BOOST_NOEXCEPT(wbc.setReference(cart_constraint.name, ref));
        BOOST_NOEXCEPT(wbc.setupOptProblem(robot_model.getTaskFrames(), opt_problem));
        BOOST_NOEXCEPT(solver.solve(opt_problem, (Eigen::VectorXd& )solver_output));

        for(int i = 0; i < 7; i++)
            cur_joint_pos[i].position += solver_output(i) * cycle_time;

        usleep(1000*1000*0.3);

        reached = (wbc.getConstraint(cart_constraint.name)->y_ref.norm() < 1e-3);
    }
}

