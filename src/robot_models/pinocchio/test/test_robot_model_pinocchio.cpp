#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "../RobotModelPinocchio.hpp"
#include "../../../core/RobotModelConfig.hpp"
#include "../../../tools/URDFTools.hpp"
#include "../../test/test_robot_model.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configure_and_update){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelPtr robot_model = std::make_shared<RobotModelPinocchio>();

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

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testFK(robot_model, tip_frame, false);
}


BOOST_AUTO_TEST_CASE(space_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, true);
}

BOOST_AUTO_TEST_CASE(body_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testBodyJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(com_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testCoMJacobian(robot_model, false);
}

BOOST_AUTO_TEST_CASE(inverse_dynamics){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testInverseDynamics(robot_model, false);

}
