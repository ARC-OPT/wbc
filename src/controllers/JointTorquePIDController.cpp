#include "JointTorquePIDController.hpp"

namespace ctrl_lib{

JointTorquePIDController::JointTorquePIDController(const std::vector<std::string>& joint_names) :
    PIDController(joint_names.size()),
    joint_names(joint_names){

    control_output_joints.resize(joint_names.size());
    control_output_joints.names = joint_names;
}

void JointTorquePIDController::extractFeedback(const base::samples::Joints& feedback){
    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            this->feedback(i) = feedback.getElementByName(joint_names[i]).effort;
        }
        catch(std::exception e){
            throw std::runtime_error("JointTorquePIDController::update: Feedback vector does not contain element " + joint_names[i]);
        }
    }
}

void JointTorquePIDController::extractSetpoint(const base::commands::Joints& setpoint){
    // For all entries where not setpoint is given set setpoint[i] = feedback[i]
    this->setpoint = this->feedback;
    for(size_t i = 0; i < setpoint.size(); i++){
        try{
            int idx = control_output_joints.mapNameToIndex(setpoint.names[i]);
            this->setpoint(idx) = setpoint[i].effort;
        }
        catch(std::exception e){
            throw std::runtime_error("JointTorquePIDController::update: Setpoint vector contains " + setpoint.names[i]
                                     + " but this element has not been configured in the controller");
        }
    }
}

const base::commands::Joints& JointTorquePIDController::update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback, const double& dt){
    extractSetpoint(setpoint);
    extractFeedback(feedback);

    PIDController::update(dt);

    for(size_t i = 0; i < joint_names.size(); i++){
        control_output_joints[i].position     = std::numeric_limits<double>::quiet_NaN();
        control_output_joints[i].speed        = control_output(i);
        control_output_joints[i].acceleration = std::numeric_limits<double>::quiet_NaN();
    }
    control_output_joints.time = base::Time::now();

    return control_output_joints;

}

}
