#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "scenes/acceleration_reduced_tsid/AccelerationSceneReducedTSID.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"
#include "tasks/SpatialAccelerationTask.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(simple_test){

    /**
     * Check if the WBC scene computes the correct result, i.e., if the reference spatial acceleration matches the solver output, back-projected to Cartesian space
     */

    // Configure Robot model
    shared_ptr<RobotModelPinocchio> robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig config;
    config.file_or_string = "../../../../../models/rh5/urdf/rh5_legs.urdf";
    config.floating_base = true;
    config.contact_points = {types::Contact("FL_SupportCenter",1,0.6,0.2,0.08), types::Contact("FR_SupportCenter",1,0.6,0.2,0.08)};
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
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure scene
    SpatialAccelerationTaskPtr cart_task;
    cart_task = make_shared<SpatialAccelerationTask>(TaskConfig("cart_pos_ctrl",0,Eigen::VectorXd::Ones(6),1),
                                                       robot_model,
                                                       "RH5_Root_Link");
    AccelerationSceneReducedTSID wbc_scene(robot_model, solver, 1e-3);
    BOOST_CHECK_EQUAL(wbc_scene.configure({cart_task}), true);

    // Set random Reference
    types::RigidBodyState ref;
    srand (time(NULL));
    ref.acceleration.linear = Eigen::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    ref.acceleration.angular = Eigen::Vector3d(((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX, ((double)rand())/RAND_MAX);
    BOOST_CHECK_NO_THROW(cart_task->setReference(ref.acceleration));

    // Solve
    HierarchicalQP qp;
    BOOST_CHECK_NO_THROW(qp=wbc_scene.update());
    types::JointCommand solver_output = wbc_scene.solve(qp);
    Eigen::VectorXd solver_output_raw = wbc_scene.getSolverOutputRaw();

    // Check
    uint nj = robot_model->nj();

    Eigen::VectorXd qdd = solver_output_raw.head(nj);
    Eigen::VectorXd tau = solver_output.effort;
    vector<types::Wrench> contact_wrenches = wbc_scene.getContactWrenches();

    Eigen::VectorXd y_solution = robot_model->spaceJacobian(cart_task->tipFrame())*qdd + robot_model->spatialAccelerationBias(cart_task->tipFrame()).vector6d();

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(ref.acceleration.linear[i] - y_solution[i]) < 1e-3);
        BOOST_CHECK(fabs(ref.acceleration.angular[i] - y_solution[i]) < 1e3);
    }

    // Check if torques respect equation of motions
    const auto& contacts = robot_model->getContacts();
    Eigen::VectorXd eq_motion_left = robot_model->jointSpaceInertiaMatrix() * qdd + robot_model->biasForces();
    Eigen::VectorXd eq_motion_right(nj);
    eq_motion_right = robot_model->selectionMatrix().transpose()*tau;
    for(uint i=0; i < contacts.size(); ++i)
        eq_motion_right += robot_model->spaceJacobian(contacts[i].frame_id).topRows(3).transpose() * contact_wrenches[i].force;


    BOOST_CHECK((eq_motion_left - eq_motion_right).cwiseAbs().maxCoeff() < 1e-3);
}
