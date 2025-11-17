#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include "../RobotModelHyrodyn.hpp"
#include <boost/test/unit_test.hpp>
#include "../../test/test_robot_model.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configure_and_update){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelPtr robot_model = std::make_shared<RobotModelHyrodyn>();

    vector<string> joint_names = {"kuka_lbr_l_joint_1",
                                  "kuka_lbr_l_joint_2",
                                  "kuka_lbr_l_joint_3",
                                  "kuka_lbr_l_joint_4",
                                  "kuka_lbr_l_joint_5",
                                  "kuka_lbr_l_joint_6",
                                  "kuka_lbr_l_joint_7"};
    std::vector<std::string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                                    "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    // Valid config
    config = RobotModelConfig("../../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    BOOST_CHECK(robot_model->configure(config) == true);

    for(size_t i = 0; i < robot_model->na(); i++){
        BOOST_CHECK(joint_names[i] == robot_model->jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model->actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model->independentJointNames()[i]);
        BOOST_CHECK(i == robot_model->jointIndex(joint_names[i]));
    }

    types::JointState joint_state = makeRandomJointState(joint_names.size());
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration));
    types::JointState joint_state_out = robot_model->jointState();
    for(size_t n = 0; n < robot_model->na(); n++){
        BOOST_CHECK(joint_state_out.position[n] == joint_state.position[n]);
        BOOST_CHECK(joint_state_out.velocity[n] == joint_state.velocity[n]);
        BOOST_CHECK(joint_state_out.acceleration[n] == joint_state.acceleration[n]);
    }

    // Valid config with floating base
    config = RobotModelConfig("../../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.floating_base = true;
    BOOST_CHECK(robot_model->configure(config) == true);

    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::RigidBodyState floating_base_state_out = robot_model->floatingBaseState();
    joint_state_out = robot_model->jointState();
    for(size_t n = 0; n < robot_model->na(); n++){
        BOOST_CHECK(joint_state_out.position[n] == joint_state.position[n]);
        BOOST_CHECK(joint_state_out.velocity[n] == joint_state.velocity[n]);
        BOOST_CHECK(joint_state_out.acceleration[n] == joint_state.acceleration[n]);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(floating_base_state.pose.position[i] == floating_base_state_out.pose.position[i]);
        BOOST_CHECK(floating_base_state.pose.orientation.coeffs()[i] == floating_base_state_out.pose.orientation.coeffs()[i]);
        BOOST_CHECK(floating_base_state.twist.linear[i] == floating_base_state_out.twist.linear[i]);
        BOOST_CHECK(floating_base_state.twist.angular[i] == floating_base_state_out.twist.angular[i]);
        BOOST_CHECK(floating_base_state.acceleration.linear[i] == floating_base_state_out.acceleration.linear[i]);
        BOOST_CHECK(floating_base_state.acceleration.angular[i] == floating_base_state_out.acceleration.angular[i]);
    }
}


BOOST_AUTO_TEST_CASE(fk){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testFK(robot_model, tip_frame, false);
}


BOOST_AUTO_TEST_CASE(space_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, true);
}

BOOST_AUTO_TEST_CASE(body_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testBodyJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(com_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testCoMJacobian(robot_model, false);
}

BOOST_AUTO_TEST_CASE(inverse_dynamics){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testInverseDynamics(robot_model, false);

}


BOOST_AUTO_TEST_CASE(compare_serial_vs_hybrid_model){

    /**
     * Check if the differential inverse kinematics solution of a serial robot and the equivalent a series-parallel hybrid robot model match
     */

    string root = "RH5_Root_Link";
    string tip  = "LLAnklePitch_Link";

    RobotModelPtr robot_model_hybrid = std::make_shared<RobotModelHyrodyn>();
    RobotModelConfig config_hybrid("../../../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf");
    config_hybrid.submechanism_file = "../../../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    BOOST_CHECK(robot_model_hybrid->configure(config_hybrid) == true);


    RobotModelPtr robot_model_serial = std::make_shared<RobotModelHyrodyn>();
    RobotModelConfig config_serial("../../../../../models/rh5/urdf/rh5_single_leg.urdf");
    config_serial.submechanism_file = "../../../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    BOOST_CHECK(robot_model_serial->configure(config_serial) == true);

    types::JointState joint_state;
    joint_state.resize(robot_model_serial->na());
    joint_state.position.setZero();
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();
    joint_state.position[4] = 1.5;
    joint_state.position[5] = -0.7;

    robot_model_serial->update(joint_state.position,
                              joint_state.velocity,
                              joint_state.acceleration);
    robot_model_hybrid->update(joint_state.position,
                              joint_state.velocity,
                              joint_state.acceleration);

    //cout<<"******************** HYBRID MODEL *****************"<<endl;
    Eigen::MatrixXd jac = robot_model_hybrid->spaceJacobian(tip);
    Eigen::VectorXd v(6);
    v.setZero();
    v[2] = -0.1;
    Eigen::VectorXd u = jac.completeOrthogonalDecomposition().pseudoInverse()*v;
    std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->hyrodynHandle()->ud = u;
    std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->hyrodynHandle()->calculate_forward_system_state();

    /*cout<< "Solution actuation space" << endl;
    std::cout<<robot_model_hybrid->hyrodynHandle()->ud.transpose()<<endl;

    cout<< "Solution projected to independent joint space" << endl;
    std::cout<<robot_model_hybrid->hyrodynHandle()->yd.transpose()<<endl;*/

    //cout<<"******************** SERIAL MODEL *****************"<<endl;
    jac = robot_model_serial->spaceJacobian(tip);
    Eigen::VectorXd yd = jac.completeOrthogonalDecomposition().pseudoInverse()*v;

    /*cout<< "Solution independent joint space" << endl;
    std::cout<<yd.transpose()<<endl;*/

    for(int i = 0; i < robot_model_hybrid->na(); i++)
        BOOST_CHECK(fabs(std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model_hybrid)->hyrodynHandle()->yd[i] - yd[i]) < 1e-6);
}
