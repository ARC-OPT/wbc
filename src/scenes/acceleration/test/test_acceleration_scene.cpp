#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "scenes/acceleration/AccelerationScene.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC scene computes the correct result, i.e., if the reference spatial velocity matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelPinocchio> robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig config;
    config.file_or_string = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    base::samples::Joints joint_state;
    joint_state.names = robot_model->jointNames();
    for(auto n : robot_model->jointNames()){
        base::JointState js;
        js.position = 0.1;
        js.speed = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state));

    // Configure solver
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure Scene
    TaskConfig cart_task;
    cart_task.name       = "cart_pos_ctrl_left";
    cart_task.type       = cart;
    cart_task.priority   = 0;
    cart_task.root       = "kuka_lbr_l_link_0";
    cart_task.tip        = "kuka_lbr_l_tcp";
    cart_task.ref_frame  = "kuka_lbr_l_link_0";
    cart_task.activation = 1;
    cart_task.weights    = vector<double>(6,1);
    AccelerationScene wbc_scene(robot_model, solver, 1e-3);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_task}), true);

    // Set random reference(robot_model, solver)
    base::samples::RigidBodyStateSE3 ref;
    srand (time(NULL));
    ref.acceleration.linear = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    ref.acceleration.angular = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    BOOST_CHECK_NO_THROW(wbc_scene.setReference(cart_task.name, ref));

    // Solve
    BOOST_CHECK_NO_THROW(wbc_scene.update());
    HierarchicalQP qp;
    wbc_scene.getHierarchicalQP(qp);
    wbc_scene.solve(qp);
    base::commands::Joints solver_output = wbc_scene.getSolverOutput();

    // Check
    wbc_scene.updateTasksStatus();
    TasksStatus status = wbc_scene.getTasksStatus();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(status[0].y_ref[i] - status[0].y_solution[i]) < 1e-4);
        BOOST_CHECK(fabs(status[0].y_ref[i+3] - status[0].y_solution[i+3]) < 1e-4);
    }
}
