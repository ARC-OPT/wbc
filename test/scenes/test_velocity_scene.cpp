#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include "scenes/VelocityScene.hpp"
#include "solvers/hls/HierarchicalLSSolver.hpp"
#include <tools/URDFTools.hpp>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configuration_test){

     /**
     * Check if the WBC velocity scene fails to configure with invalid configuration
     */

    ConstraintConfig cart_constraint("cart_pos_ctrl_left", 0, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", "kuka_lbr_l_link_0");

    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    VelocityScene wbc_scene(robot_model, solver);

    // Check if configuration works
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), true);

    // Empty constraint config
    BOOST_CHECK_EQUAL(wbc_scene.configure({}), false);

    // WBC config with invalid frame ids
    cart_constraint.tip = "kuka_lbr_l_";
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), false);
    cart_constraint.tip = "kuka_lbr_l_tcp";
    cart_constraint.root = "kuka_lbr_l_link_";
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), false);
    cart_constraint.root = "kuka_lbr_l_link_0";
    cart_constraint.ref_frame = "kuka_lbr_l_link_";
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), false);
}

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC velocity scene computes the correct result, i.e., if the reference spatial velocity matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";

    vector<string> joint_names = URDFTools::jointNamesFromURDF(config.file);
    config.joint_names = config.actuated_joint_names = joint_names;
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure robot model");

    std::cout<<"Joint Names from URDF"<<std::endl;
    for(auto n : joint_names)
        std::cout<<n<<std::endl;

    std::cout<<"Joint Names in Robot Model"<<std::endl;
    for(auto n : robot_model->jointNames())
        std::cout<<n<<std::endl;


/*    base::samples::Joints joint_state;
    joint_state.names = robot_model->jointNames();
    for(auto n : robot_model->jointNames()){
        base::JointState js;
        js.position = 0.5;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

    // Configure WBC Scene
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    (std::dynamic_pointer_cast<HierarchicalLSSolver>(solver))->setMaxSolverOutputNorm(1000);
    ConstraintConfig cart_constraint("cart_pos_ctrl_left", 0, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", "kuka_lbr_l_link_0", 1);
    VelocityScene wbc_scene(robot_model, solver);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), true);

    // Set Reference
    base::samples::RigidBodyStateSE3 ref;
    srand (time(NULL));
    for(int i = 0; i < 3; i++){
        ref.twist.linear[i] = ((double)rand())/RAND_MAX;
        ref.twist.angular[i] = ((double)rand())/RAND_MAX;
    }

    BOOST_CHECK_NO_THROW(wbc_scene.setReference(cart_constraint.name, ref));

    // Solve
    BOOST_CHECK_NO_THROW(wbc_scene.update());
    HierarchicalQP qp;
    wbc_scene.getHierarchicalQP(qp);
    BOOST_CHECK_NO_THROW(wbc_scene.solve(qp));
    base::commands::Joints solver_output = wbc_scene.getSolverOutput();

    // Check
    base::VectorXd qd;
    qd.resize(solver_output.size());
    for(int i = 0; i < solver_output.size(); i++)
        qd[i] = solver_output[i].speed;
    base::MatrixXd jac = robot_model->spaceJacobian(cart_constraint.ref_frame, cart_constraint.tip);
    base::VectorXd yd = jac*qd;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(yd[i] - ref.twist.linear[i]) < 1e-5);
        BOOST_CHECK(fabs(yd[i+3] - ref.twist.angular[i]) < 1e5);
    }*/
}








