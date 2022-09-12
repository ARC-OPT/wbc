#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "core/RobotModelConfig.hpp"
#include "tools/URDFTools.hpp"
#include "../test_robot_model.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(configure_and_update){

    /**
     * Verify that the robot model fails to configure with invalid configurations
     */

    RobotModelConfig config;
    RobotModelPinocchio robot_model;

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
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    BOOST_CHECK(robot_model.configure(config) == true);

    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.independentJointNames()[i]);
        BOOST_CHECK(i == robot_model.jointIndex(joint_names[i]));
    }

    base::samples::Joints joint_state_in = makeRandomJointState(joint_names);
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in));
    base::samples::Joints joint_state_out = robot_model.jointState(joint_names);
    for(auto n : joint_names){
        BOOST_CHECK(joint_state_out[n].position = joint_state_in[n].position);
        BOOST_CHECK(joint_state_out[n].speed = joint_state_in[n].speed);
        BOOST_CHECK(joint_state_out[n].acceleration = joint_state_in[n].acceleration);
    }

    // Valid config with floating base
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    BOOST_CHECK(robot_model.configure(config) == true);
    for(size_t i = 0; i < 6; i++){
        BOOST_CHECK(floating_base_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(floating_base_names[i] == robot_model.independentJointNames()[i]);
        BOOST_CHECK(i == robot_model.jointIndex(floating_base_names[i]));
    }
    for(size_t i = 0; i < robot_model.noOfActuatedJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i+6]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.independentJointNames()[i+6]);
        BOOST_CHECK(i+6 == robot_model.jointIndex(joint_names[i]));
    }

    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 floating_base_state_out = robot_model.floatingBaseState();
    joint_state_out = robot_model.jointState(joint_names);
    for(auto n : joint_names){
        BOOST_CHECK(joint_state_out[n].position = joint_state_in[n].position);
        BOOST_CHECK(joint_state_out[n].speed = joint_state_in[n].speed);
        BOOST_CHECK(joint_state_out[n].acceleration = joint_state_in[n].acceleration);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(floating_base_state_in.pose.position[i] == floating_base_state_out.pose.position[i]);
        BOOST_CHECK(floating_base_state_in.pose.orientation.coeffs()[i] == floating_base_state_out.pose.orientation.coeffs()[i]);
        BOOST_CHECK(floating_base_state_in.twist.linear[i] == floating_base_state_out.twist.linear[i]);
        BOOST_CHECK(floating_base_state_in.twist.angular[i] == floating_base_state_out.twist.angular[i]);
        BOOST_CHECK(floating_base_state_in.acceleration.linear[i] == floating_base_state_out.acceleration.linear[i]);
        BOOST_CHECK(floating_base_state_in.acceleration.angular[i] == floating_base_state_out.acceleration.angular[i]);
    }
}


BOOST_AUTO_TEST_CASE(fk){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, false);
}


BOOST_AUTO_TEST_CASE(space_jacobian){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(body_jacobian){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testBodyJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(com_jacobian){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testCoMJacobian(robot_model, false);
}

BOOST_AUTO_TEST_CASE(dynamics){

    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";

    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testDynamics(robot_model, false);

}
