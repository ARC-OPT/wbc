#ifndef WBC_CONTROLLERS_JOINTTORQUECONTROLLER_HPP
#define WBC_CONTROLLERS_JOINTTORQUECONTROLLER_HPP

#include "PIDController.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib{

class JointTorqueController : public PIDController{
protected:
    base::commands::Joints control_output_joints;
    std::vector<std::string> joint_names;

    void extractFeedback(const base::samples::Joints& feedback);
    void extractSetpoint(const base::commands::Joints& setpoint);
public:
    JointTorqueController(const std::vector<std::string>& joint_names);

    /** Convert typed to raw input data and call PIDController::update()*/
    const base::commands::Joints& update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback, const double& dt);

};

}

#endif // WBC_CONTROLLERS_JOINTTORQUECONTROLLER_HPP
