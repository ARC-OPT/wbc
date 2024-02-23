#ifndef TEST_ROBOT_MODEL_HPP
#define TEST_ROBOT_MODEL_HPP

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>
#include <wbc/core/RobotModel.hpp>

namespace wbc {
void printRbs(base::samples::RigidBodyStateSE3 rbs);
base::samples::Joints makeRandomJointState(std::vector<std::string> joint_names);
base::samples::RigidBodyStateSE3 makeRandomFloatingBaseState();
void testFK(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testSpaceJacobian(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testBodyJacobian(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testCoMJacobian(RobotModelPtr robot_model, bool verbose=false);
void testDynamics(RobotModelPtr robot_model, bool verbose);
}
#endif
