#include "WrenchPIDController.hpp"

namespace wbc {

WrenchPIDController::WrenchPIDController() :
    PIDController(6){

}

const Eigen::VectorXd WrenchPIDController::wrenchToRaw(const types::Wrench& wrench, Eigen::VectorXd& raw){
    raw.resize(6);
    raw.segment(0,3) = wrench.force;
    raw.segment(3,3) = wrench.torque;
    return raw;
}

const types::RigidBodyState& WrenchPIDController::update(const types::Wrench& setpoint, const types::Wrench& feedback, const double dt){

    wrenchToRaw(setpoint, this->setpoint);
    wrenchToRaw(feedback, this->feedback);

    PIDController::update(dt);

    control_output_wrench.twist.linear  = control_output.segment(0,3);
    control_output_wrench.twist.angular = control_output.segment(3,3);

    return control_output_wrench;
}

}
