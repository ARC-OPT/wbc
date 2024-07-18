#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "scenes/velocity_qp/VelocitySceneQP.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"
#include "tasks/CartesianVelocityTask.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC scene computes the correct result, i.e., if the reference spatial velocity matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelPinocchio> robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig config;
    config.file_or_string = "../../../../../models/rh5/urdf/rh5_legs.urdf";
    config.floating_base = true;
    config.contact_points = {Contact("FL_SupportCenter",1,0.6),Contact("FR_SupportCenter",1,0.6)};
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    types::JointState joint_state;
    joint_state.resize(robot_model->na());
    joint_state.position << 0,0,-0.35,0.64,0,-0.27,  0,0,-0.35,0.64,0,-0.27;
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();

    types::RigidBodyState rbs;
    rbs.pose.position = Eigen::Vector3d(-0.175,0,0.876);
    rbs.pose.orientation.setIdentity();
    rbs.twist.setZero();
    rbs.acceleration.setZero();

    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             rbs.pose,
                                             rbs.twist,
                                             rbs.acceleration));

    // Configure Solver
    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(10000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;    
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure scene
    CartesianVelocityTaskPtr cart_task;
    cart_task = make_shared<CartesianVelocityTask>(TaskConfig("cart_pos_ctrl", 0, {1,1,1,1,1,1},1),
                                                   "RH5_Root_Link",
                                                   "world",
                                                   robot_model->nj());

    VelocitySceneQP wbc_scene(robot_model, solver, 1e-3);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_task}), true);

    // Set random Reference
    types::RigidBodyState ref;
    srand (time(NULL));
    ref.twist.linear = Eigen::Vector3d(((double)rand())/(10.0*RAND_MAX), ((double)rand())/(10.0*RAND_MAX), ((double)rand())/(10.0*RAND_MAX));
    ref.twist.angular = Eigen::Vector3d(((double)rand())/(10.0*RAND_MAX), ((double)rand())/(10.0*RAND_MAX), ((double)rand())/(10.0*RAND_MAX));
    cart_task->setReference(ref.twist);

    // Solve
    BOOST_CHECK_NO_THROW(wbc_scene.update());
    HierarchicalQP qp;
    wbc_scene.getHierarchicalQP(qp);
    wbc_scene.solve(qp);

    // Check
    Eigen::VectorXd y_solution(6);
    Eigen::VectorXd solver_output = wbc_scene.getSolverOutputRaw();
    y_solution = robot_model->spaceJacobian(cart_task->tipFrame())*solver_output;
    std::cout<<y_solution.transpose()<<std::endl;
    std::cout<<ref.twist.linear.transpose()<<" "<<ref.twist.angular.transpose()<<std::endl;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(ref.twist.linear[i] - y_solution[i]) < 1e-3);
        BOOST_CHECK(fabs(ref.twist.angular[i] - y_solution[i+3]) < 1e3);
    }
}
