#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "scenes/VelocityScene.hpp"
#include "solvers/hls/HierarchicalLSSolver.hpp"

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
    BOOST_CHECK(robot_model->configure(config));

    base::samples::Joints joint_state;
    joint_state.names = robot_model->jointNames();
    for(auto n : robot_model->jointNames()){
        base::JointState js;
        js.position = 0.5;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

    // Configure solver
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    (std::dynamic_pointer_cast<HierarchicalLSSolver>(solver))->setMaxSolverOutputNorm(1000);

    // Configure scene
    VelocityScene wbc_scene(robot_model, solver);
    ConstraintConfig cart_constraint("cart_pos_ctrl_left", 0, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", "kuka_lbr_l_link_0", 1);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), true);

    // Set random Reference
    base::samples::RigidBodyStateSE3 ref;
    srand (time(NULL));
    ref.twist.linear = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    ref.twist.angular = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    BOOST_CHECK_NO_THROW(wbc_scene.setReference(cart_constraint.name, ref));

    // Solve
    BOOST_CHECK_NO_THROW(wbc_scene.update());
    HierarchicalQP qp;
    wbc_scene.getHierarchicalQP(qp);
    BOOST_CHECK_NO_THROW(wbc_scene.solve(qp));
    base::commands::Joints solver_output = wbc_scene.getSolverOutput();

    // Check
    wbc_scene.updateConstraintsStatus();
    ConstraintsStatus status = wbc_scene.getConstraintsStatus();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(status[0].y_ref[i] - status[0].y_solution[i]) < 1e-5);
        BOOST_CHECK(fabs(status[0].y_ref[i+3] - status[0].y_solution[i]) < 1e5);
    }
}








