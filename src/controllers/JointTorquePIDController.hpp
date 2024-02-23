#ifndef WBC_CONTROLLERS_JOINTTORQUEPIDCONTROLLER_HPP
#define WBC_CONTROLLERS_JOINTTORQUEPIDCONTROLLER_HPP

#include "PIDController.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{

class JointTorquePIDController : public PIDController{
protected:
    base::commands::Joints control_output_joints;
    std::vector<std::string> joint_names;

    void extractFeedback(const base::samples::Joints& feedback);
    void extractSetpoint(const base::commands::Joints& setpoint);
public:
    JointTorquePIDController(const std::vector<std::string>& joint_names);
    virtual ~JointTorquePIDController(){}

    /** Convert typed to raw input data and call PIDController::update()*/
    const base::commands::Joints& update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback, const double& dt);

};

}

#endif // WBC_CONTROLLERS_JOINTTORQUEPIDCONTROLLER_HPP
