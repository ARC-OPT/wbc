#include "JointPosPDController.hpp"

namespace ctrl_lib{

JointPosPDController::JointPosPDController(const std::vector<std::string>& joint_names) :
    PosPDController(joint_names.size()),
    joint_names(joint_names){

    control_output.resize(joint_names.size());
    control_output.names = joint_names;
}

void JointPosPDController::extractFeedback(const base::samples::Joints& feedback){
        for(size_t i = 0; i < joint_names.size(); i++){
            base::JointState joint_state;
            try{
                joint_state = feedback.getElementByName(joint_names[i]);
            }
            catch(std::exception e){
                throw std::runtime_error("JointPosPDController::update: Feedback vector does not contain element " + joint_names[i]);
            }
            pos(i) = joint_state.position;
            vel(i) = joint_state.speed;
            acc(i) = joint_state.acceleration;
        }
}

void JointPosPDController::extractSetpoint(const base::commands::Joints& setpoint){
    // If a setpoint value is not given, set setpoint == feedback
    ref_pos = pos;
    ref_vel = vel;
    for(size_t i = 0; i < joint_names.size(); i++){
        base::JointState joint_state;
        try{
            joint_state = setpoint.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            throw std::runtime_error("JointPosPDController::update: Setpoint vector contains " + setpoint.names[i]
                                     + " but this element has not been configured in the controller");
        }
        ref_pos(i) = joint_state.position;
        ref_vel(i) = joint_state.speed;
        ref_acc(i) = joint_state.acceleration;
    }
}

const base::commands::Joints& JointPosPDController::update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback){
    extractFeedback(feedback);
    extractSetpoint(setpoint);

    PosPDController::update();

    for(size_t i = 0; i < joint_names.size(); i++){
        control_output[i].position     = std::numeric_limits<double>::quiet_NaN();
        control_output[i].speed        = control_out_vel(i);
        control_output[i].acceleration = control_out_acc(i);
    }
    control_output.time = base::Time::now();

    return control_output;
}
}
