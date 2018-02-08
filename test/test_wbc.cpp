#include <boost/test/unit_test.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <iostream>
#include "KinematicRobotModelKDL.hpp"
#include "RobotModelConfig.hpp"
#include "HierarchicalLSSolver.hpp"
#include "Jacobian.hpp"
#include "LinearEqualityConstraints.hpp"
#include "ConstraintConfig.hpp"
#include "WbcVelocityScene.hpp"

using namespace std;
using namespace wbc;

void pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b, const double dt, base::Vector6d& twist){

    Eigen::Matrix3d rot_mat = a.rotation().inverse() * b.rotation();
    Eigen::AngleAxisd angle_axis;
    angle_axis.fromRotationMatrix(rot_mat);

    twist.segment(0,3) = (b.translation() - a.translation())/dt;
    twist.segment(3,3) = a.rotation() * (angle_axis.axis() * angle_axis.angle())/dt;
}

void pose_diff(const base::samples::RigidBodyState& a, const base::samples::RigidBodyState& b, const double dt, base::Vector6d& twist){
    pose_diff(a.getTransform(), b.getTransform(), dt, twist);
}

BOOST_AUTO_TEST_CASE(test_jacobian)
{
    cout<<"Testing Jacobian ...."<<endl<<endl;

    srand(time(NULL));

    cout<<"Testing change Ref point ..."<<endl<<endl;

    Jacobian jac(7);
    jac.setIdentity();
    jac.changeRefPoint(base::Vector3d(0.1,0.2,0.3));

    KDL::Jacobian jac_kdl(7);
    jac_kdl.data.setIdentity();
    jac_kdl.changeRefPoint(KDL::Vector(0.1,0.2,0.3));

    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 7; j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    cout<<"Jacobian from WBC is " <<endl;
    cout<<jac<<endl;

    cout<<"Jacobian from KDL is " <<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"Testing change Ref Frame ..."<<endl<<endl;

    base::Affine3d a;
    a.setIdentity();
    a.translate(base::Vector3d(1,2,3));
    jac.changeRefFrame(a);

    KDL::Frame a_kdl;
    a_kdl.p = KDL::Vector(1,2,3);
    jac_kdl.changeRefFrame(a_kdl);

    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 7; j++)
            BOOST_CHECK_EQUAL(jac(i,j), jac_kdl.data(i,j));

    cout<<"Jacobian from WBC is " <<endl;
    cout<<jac<<endl;

    cout<<"Jacobian from KDL is " <<endl;
    cout<<jac_kdl.data<<endl<<endl;

    cout<<"...done"<<endl<<endl;
}

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


BOOST_AUTO_TEST_CASE(hierarchical_ls_solver)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 2;
    const double NORM_MAX = 5.75;
    const double MIN_EIGENVALUE = 1e-9;

    HierarchicalLSSolver solver;
    vector<int> ny_per_prio(1,NO_CONSTRAINTS);

    BOOST_CHECK_EQUAL(solver.configure(ny_per_prio, NO_JOINTS), true);

    solver.setMaxSolverOutputNorm(NORM_MAX);
    solver.setMinEigenvalue(MIN_EIGENVALUE);

    std::vector<LinearEqualityConstraints> input;
    LinearEqualityConstraints prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        prio_0.A.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.y_ref.data()[i] = (rand()%1000)/1000.0;
    prio_0.Wq.setConstant(1);

    input.push_back(prio_0);

    cout<<"\n----------------------- Testing Hierarchical Solver ----------------------"<<endl<<endl;
    cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
    cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;

    cout<<"\nSolver Input ..."<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"Priority: "<<i<<endl;
        cout<<"Constraint Matrix: "<<endl; cout<<prio_0.A<<endl;
        cout<<"Reference: "<<endl; cout<<prio_0.y_ref<<endl;
        cout<<endl;
    }

    base::VectorXd solver_output;
    solver.solve(input, solver_output);

    cout<<"\nSolver Output ..."<<endl;
    cout<<"q_dot = "<<endl;
    cout<<solver_output<<endl;

    cout<<"\nTEST..."<<endl<<endl;
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

BOOST_AUTO_TEST_CASE(wbc_velocity_scene){

    vector<string> joint_names;
    for(int i = 0; i < 7; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));

    // Create WBC config
    vector<ConstraintConfig> wbc_config;

    // Constraint for Cartesian Position Control
    ConstraintConfig cart_constraint;
    cart_constraint.name       = "cart_pos_ctrl_left";
    cart_constraint.type       = cart;
    cart_constraint.priority   = 0;
    cart_constraint.root       = "kuka_lbr_base";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_base";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    // Configure Robot model
    shared_ptr<KinematicRobotModelKDL> robot_model = make_shared<KinematicRobotModelKDL>();
    vector<RobotModelConfig> config(1);
    config[0].file = std::string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/kuka_lbr.urdf";
    BOOST_CHECK_EQUAL(robot_model->configure(config, joint_names, "kuka_lbr_base"), true);

    // Configure Solver
    shared_ptr<HierarchicalLSSolver> solver = make_shared<HierarchicalLSSolver>();
    BOOST_CHECK_EQUAL(solver->configure(WbcScene::getNConstraintVariablesPerPrio(wbc_config), robot_model->noOfJoints()), true);

    // Configure WBC Scene
    WbcVelocityScene wbc_scene(robot_model);
    BOOST_CHECK_EQUAL(wbc_scene.configure(wbc_config), true);

    // Set reference
    base::samples::RigidBodyState target, ref, act;
    target.time = base::Time::now();
    target.position = base::Vector3d(0.235498, 0.822147, 1.41881);
    target.orientation.setIdentity();
    act.position.setZero();
    act.orientation.setIdentity();

    // Run control loop
    base::samples::Joints joint_state;
    joint_state.resize(7);
    joint_state.names = robot_model->jointNames();
    for(int i = 0; i < 7; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now();

    base::VectorXd solver_output;

    double loop_time = 0.1; // seconds
    while((target.position - act.position).norm() > 1e-4){

        // Update robot model
        BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

        act = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        base::Vector6d diff;
        pose_diff(act, target, 1, diff);
        ref.velocity = diff.segment(0,3);
        ref.angular_velocity = diff.segment(3,3);
        ref.time = base::Time::now();
        shared_ptr<CartesianVelocityConstraint> constraint = static_pointer_cast<CartesianVelocityConstraint>(wbc_scene.getConstraint("cart_pos_ctrl_left"));
        constraint->setReference(ref);

        // Compute ctrl solution
        wbc_scene.update();
        solver->solve(wbc_scene.getHierarhicalLEConstraints(), solver_output);

        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += solver_output(i) * loop_time;

        cout<<"Target: x: "<<target.position(0)<<" y: "<<target.position(1)<<" z: "<<target.position(2)<<endl;
        cout<<"Target: qx: "<<target.orientation.x()<<" qy: "<<target.orientation.y()<<" qz: "<<target.orientation.z()<<" qw: "<<target.orientation.w()<<endl<<endl;

        cout<<"Actual x: "<<act.position(0)<<" y: "<<act.position(1)<<" z: "<<act.position(2)<<endl;
        cout<<"Actual qx: "<<act.orientation.x()<<" qy: "<<act.orientation.y()<<" qz: "<<act.orientation.z()<<" qw: "<<act.orientation.w()<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }
}
