#ifndef ROBOT_MODEL_COMMON_HPP
#define ROBOT_MODEL_COMMON_HPP

#include <core/RobotModel.hpp>

namespace wbc{

RobotModelPtr makeRobotModelKUKAIiwa(const std::string type);
RobotModelPtr makeRobotModelRH5SingleLeg(const std::string type, bool hybrid_model = false);
RobotModelPtr makeRobotModelRH5Legs(const std::string type, bool hybrid_model = false);
RobotModelPtr makeRobotModelRH5(const std::string type, bool hybrid_model = false);
RobotModelPtr makeRobotModelRH5v2(const std::string type, bool hybrid_model = false);

}

#endif
