#include "CartesianPosPDController.hpp"
#include "ControllerTools.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace ctrl_lib{

CartesianPosPDController::CartesianPosPDController() :
    PosPDController(6){

    control_output.setNaN();
}

void CartesianPosPDController::extractFeedback(const base::samples::RigidBodyStateSE3& feedback){
    pos.setZero();
    vel.segment(0,3) = feedback.twist.linear;
    vel.segment(3,3) = feedback.twist.angular;
    acc.segment(0,3) = feedback.acceleration.linear;
    acc.segment(3,3) = feedback.acceleration.angular;
}

void CartesianPosPDController::extractSetpoint(const base::samples::RigidBodyStateSE3& setpoint, const base::samples::RigidBodyStateSE3& feedback){
    pose_diff = setpoint.pose - feedback.pose;
    ref_pos.segment(0,3) = pose_diff.linear;
    ref_pos.segment(3,3) = pose_diff.angular;
    ref_vel.segment(0,3) = setpoint.twist.linear;
    ref_vel.segment(3,3) = setpoint.twist.angular;
    ref_acc.segment(0,3) = setpoint.acceleration.linear;
    ref_acc.segment(3,3) = setpoint.acceleration.angular;
}

const base::samples::RigidBodyStateSE3& CartesianPosPDController::update(const base::samples::RigidBodyStateSE3 &setpoint, const base::samples::RigidBodyStateSE3 &feedback){

    extractFeedback(feedback);
    extractSetpoint(setpoint, feedback);

    PosPDController::update();

    control_output.twist.linear  = control_out_vel.segment(0,3);
    control_output.twist.angular = control_out_vel.segment(3,3);
    control_output.acceleration.linear  = control_out_acc.segment(0,3);
    control_output.acceleration.angular = control_out_acc.segment(3,3);
    control_output.frame_id = setpoint.frame_id;
    control_output.time = base::Time::now();

    return control_output;
}
}
