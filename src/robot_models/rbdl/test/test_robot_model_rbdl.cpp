#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "tools/URDFTools.hpp"
#include "robot_models/rbdl/RobotModelRBDL.hpp"
#include "../../test/test_robot_model.hpp"

using namespace std;
using namespace wbc;
using namespace RigidBodyDynamics;

BOOST_AUTO_TEST_CASE(configure_and_update){

    srand(time(NULL));

    vector<string> joint_names = {"BodyPitch","BodyRoll","BodyYaw",
                                  "ALShoulder1","ALShoulder2","ALShoulder3","ALElbow","ALWristRoll","ALWristYaw","ALWristPitch",
                                  "ARShoulder1","ARShoulder2","ARShoulder3","ARElbow","ARWristRoll","ARWristYaw","ARWristPitch",
                                  "HeadPitch","HeadRoll","HeadYaw",
                                  "LLHip1","LLHip2","LLHip3","LLKnee","LLAnkleRoll","LLAnklePitch",
                                  "LRHip1","LRHip2","LRHip3","LRKnee","LRAnkleRoll","LRAnklePitch"};
    // Valid config
    RobotModelRBDL robot_model;
    RobotModelConfig cfg("../../../../../models/rh5/urdf/rh5.urdf");
    cfg.floating_base = true;
    BOOST_CHECK(robot_model.configure(cfg));
    types::JointState joint_state = makeRandomJointState(joint_names.size());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state.position,
                                            joint_state.velocity,
                                            joint_state.acceleration,
                                            floating_base_state.pose,
                                            floating_base_state.twist,
                                            floating_base_state.acceleration));

    types::RigidBodyState floating_base_state_out = robot_model.floatingBaseState();
    types::JointState joint_state_out = robot_model.jointState();
    for(size_t n = 0; n < robot_model.na(); n++){
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

    RobotModelPtr robot_model = make_shared<RobotModelRBDL>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(space_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelRBDL>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testSpaceJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(body_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelRBDL>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testBodyJacobian(robot_model, tip_frame, false);
}

BOOST_AUTO_TEST_CASE(com_jacobian){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelRBDL>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = false;
    BOOST_CHECK(robot_model->configure(cfg));

    testCoMJacobian(robot_model, false);
}

BOOST_AUTO_TEST_CASE(inverse_dynamics){
    string urdf_file = "../../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelPtr robot_model = make_shared<RobotModelRBDL>();
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model->configure(cfg));

    testInverseDynamics(robot_model, false);

}
