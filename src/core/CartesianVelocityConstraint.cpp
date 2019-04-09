#include "CartesianVelocityConstraint.hpp"
#include <base-logging/Logging.hpp>
#include "../types/CartesianState.hpp"

namespace wbc{

CartesianVelocityConstraint::CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

CartesianVelocityConstraint::~CartesianVelocityConstraint(){
}

void CartesianVelocityConstraint::setReference(const CartesianState& ref){

    if(!ref.hasValidTwist()){
        LOG_ERROR("Constraint %s has invalid velocity and/or angular velocity", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    this->time = ref.time;
    this->y_ref.segment(0,3) = ref.twist.linear;
    this->y_ref.segment(3,3) = ref.twist.angular;
}

} // namespace wbc
