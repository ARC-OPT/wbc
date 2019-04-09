#include "CartesianPosPDController.hpp"
#include "../types/CartesianState.hpp"

using namespace wbc;

namespace ctrl_lib{

CartesianPosPDController::CartesianPosPDController() :
    PosPDController(6){

    control_output.setNaN();
}

void CartesianPosPDController::extractFeedback(const wbc::CartesianState& feedback){
    pos.setZero();
    vel.segment(0,3) = feedback.twist.linear;
    vel.segment(3,3) = feedback.twist.angular;
}

void CartesianPosPDController::extractSetpoint(const wbc::CartesianState& setpoint, const wbc::CartesianState& feedback){
    pose_diff = setpoint.pose - feedback.pose;
    ref_pos.segment(0,3) = pose_diff.linear;
    ref_pos.segment(3,3) = pose_diff.angular;
    ref_vel.segment(0,3) = setpoint.twist.linear;
    ref_vel.segment(3,3) = setpoint.twist.angular;
    ref_acc.segment(0,3) = setpoint.acceleration.linear;
    ref_acc.segment(3,3) = setpoint.acceleration.angular;
}

const wbc::CartesianState& CartesianPosPDController::update(const wbc::CartesianState &setpoint, const wbc::CartesianState &feedback){

    extractFeedback(feedback);
    extractSetpoint(setpoint, feedback);

    PosPDController::update();

    control_output.twist.linear  = control_out_vel.segment(0,3);
    control_output.twist.angular = control_out_vel.segment(3,3);
    control_output.acceleration.linear  = control_out_acc.segment(0,3);
    control_output.acceleration.angular = control_out_acc.segment(3,3);
    control_output.source_frame = setpoint.source_frame;
    control_output.target_frame = setpoint.target_frame;
    control_output.time = base::Time::now();

    return control_output;
}
}
