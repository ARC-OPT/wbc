#ifndef TEST_ROBOT_MODEL_HPP
#define TEST_ROBOT_MODEL_HPP

#include "../../types/RigidBodyState.hpp"
#include "../../types/JointState.hpp"
#include "../../core/RobotModel.hpp"

namespace wbc {
void printRbs(types::RigidBodyState rbs);
types::JointState makeRandomJointState(uint n);
types::RigidBodyState makeRandomFloatingBaseState();
void testFK(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testSpaceJacobian(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testBodyJacobian(RobotModelPtr robot_model, const std::string &tip_frame, bool verbose=false);
void testCoMJacobian(RobotModelPtr robot_model, bool verbose=false);
void testDynamics(RobotModelPtr robot_model, bool verbose);
void testInverseDynamics(RobotModelPtr robot_model, bool verbose);
}
#endif
