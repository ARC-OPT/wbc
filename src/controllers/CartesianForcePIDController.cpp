#include "CartesianForcePIDController.hpp"

namespace wbc {

CartesianForcePIDController::CartesianForcePIDController() :
    PIDController(6){

}

const base::VectorXd CartesianForcePIDController::wrenchToRaw(const base::samples::Wrench& wrench, base::VectorXd& raw){
    raw.resize(6);
    raw.segment(0,3) = wrench.force;
    raw.segment(3,3) = wrench.torque;
    return raw;
}

const base::samples::RigidBodyStateSE3& CartesianForcePIDController::update(const base::samples::Wrench& setpoint, const base::samples::Wrench& feedback, const double dt){

    wrenchToRaw(setpoint, this->setpoint);
    wrenchToRaw(feedback, this->feedback);

    PIDController::update(dt);

    control_output_wrench.twist.linear  = control_output.segment(0,3);
    control_output_wrench.twist.angular = control_output.segment(3,3);
    control_output_wrench.time = base::Time::now();

    return control_output_wrench;
}

}
