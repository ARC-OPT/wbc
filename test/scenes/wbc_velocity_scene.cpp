#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include "scenes/VelocityScene.hpp"
#include "solvers/hls/HierarchicalLSSolver.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(simple_test){

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
    cart_constraint.root       = "kuka_lbr_l_link_0";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_l_link_0";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    // Configure Robot model
    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    config.joint_names = joint_names;
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    base::samples::Joints joint_state;
    joint_state.names = joint_names;
    for(auto n : joint_names){
        base::JointState js;
        js.position = rand()/RAND_MAX;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

    // Configure WBC Scene
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    VelocityScene wbc_scene(robot_model, solver);
    BOOST_CHECK_EQUAL(wbc_scene.configure(wbc_config), true);

    // Set Reference
    base::samples::RigidBodyStateSE3 ref;
    for(int i = 0; i < 3; i++){
        ref.twist.linear[i] = rand()/RAND_MAX;
        ref.twist.angular[i] = rand()/RAND_MAX;
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
        BOOST_CHECK_EQUAL(yd[i], ref.twist.linear[i]);
        BOOST_CHECK_EQUAL(yd[i+3], ref.twist.angular[i]);
    }

}
