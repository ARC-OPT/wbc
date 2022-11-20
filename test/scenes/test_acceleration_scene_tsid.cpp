#include <boost/test/unit_test.hpp>
#include "robot_models/kdl/RobotModelKDL.hpp"
#include "scenes/AccelerationSceneTSID.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC scene computes the correct result, i.e., if the reference spatial acceleration matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../models/rh5/urdf/rh5_legs.urdf";
    config.floating_base = true;
    config.contact_points.names = {"FL_SupportCenter", "FR_SupportCenter"};
    wbc::ActiveContact contact(1,0.6);
    contact.wx = 0.2;
    contact.wy = 0.08;
    config.contact_points.elements = {contact, contact};
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    vector<double> q_in = {0,0,-0.35,0.64,0,-0.27,
                           0,0,-0.35,0.64,0,-0.27};

    base::samples::Joints joint_state;
    joint_state.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        base::JointState js;
        js.position = q_in[i];
        js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();

    base::samples::RigidBodyStateSE3 rbs;
    rbs.pose.position = base::Vector3d(-0.175,0,0.876);
    rbs.pose.orientation.setIdentity();
    rbs.twist.setZero();
    rbs.acceleration.setZero();
    rbs.time = base::Time::now();

    BOOST_CHECK_NO_THROW(robot_model->update(joint_state,rbs));

    // Configure Solver
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure scene
    TaskConfig cart_task;
    cart_task.type = cart;
    cart_task.name = "cart_pos_ctrl";
    cart_task.root = "world";
    cart_task.tip = "RH5_Root_Link";
    cart_task.ref_frame = "world";
    cart_task.weights = {1,1,1,1,1,1};
    cart_task.priority = 0;
    cart_task.activation = 1;
    AccelerationSceneTSID wbc_scene(robot_model, solver);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_task}), true);

    // Set random Reference
    base::samples::RigidBodyStateSE3 ref;
    srand (time(NULL));
    ref.acceleration.linear = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    ref.acceleration.linear.setZero();
    ref.acceleration.angular = base::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    ref.acceleration.angular.setZero();
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
        BOOST_CHECK(fabs(status[0].y_ref[i] - status[0].y_solution[i]) < 1e-3);
        BOOST_CHECK(fabs(status[0].y_ref[i+3] - status[0].y_solution[i+3]) < 1e3);
    }
}
