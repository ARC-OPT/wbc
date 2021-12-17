#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include "scenes/VelocitySceneQuadraticCost.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"

using namespace std;
using namespace wbc;

string rootDir(){
    std::string root_dir = string(__FILE__);
    const size_t last_slash_idx = root_dir.rfind('/');
    root_dir =  root_dir.substr(0, last_slash_idx) + "/../..";
    return root_dir;
}

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC scene computes the correct result, i.e., if the reference spatial velocity matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = rootDir() + "/models/kuka/urdf/kuka_iiwa.urdf";
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    base::samples::Joints joint_state;
    joint_state.names = robot_model->jointNames();
    for(auto n : robot_model->jointNames()){
        base::JointState js;
        js.position = 0.5;
        js.speed = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

    // Configure WBC Scene
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    ConstraintConfig cart_constraint("cart_pos_ctrl_left", 0, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", "kuka_lbr_l_link_0", 1);
    VelocitySceneQuadraticCost wbc_scene(robot_model, solver);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_constraint}), true);

    // Set Reference
    base::samples::RigidBodyStateSE3 ref;
    srand(time(NULL));
    for(int i = 0; i < 3; i++){
        ref.twist.linear[i] = ((double)rand())/RAND_MAX;
        ref.twist.angular[i] = ((double)rand())/RAND_MAX;
    }

    BOOST_CHECK_NO_THROW(wbc_scene.setReference(cart_constraint.name, ref));

    // Solve
    BOOST_CHECK_NO_THROW(wbc_scene.update());
    HierarchicalQP qp;
    wbc_scene.getHierarchicalQP(qp);
    qp[0].lower_x.resize(0);
    qp[0].upper_x.resize(0);
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
    }
}
