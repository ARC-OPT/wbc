#ifndef CTRL_LIB_JOINT_POS_PD_CONTROLLER_HPP
#define CTRL_LIB_JOINT_POS_PD_CONTROLLER_HPP

#include "PosPDController.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

/**
 * @brief The JointPosPDController class implements the PosPDController in joint space
 */
class JointPosPDController : public PosPDController{
protected:
    base::commands::Joints control_output;
    std::vector<std::string> joint_names;

    void extractFeedback(const base::samples::Joints& feedback);
    void extractSetpoint(const base::commands::Joints& setpoint);

public:
    JointPosPDController(const std::vector<std::string>& joint_names);

    /** Convert typed to raw input data and call PosPDController::update()*/
    const base::commands::Joints& update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback);
};

}
#endif
