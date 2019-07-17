#include "CartesianAccelerationConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <ctrl_types/CartesianState.hpp>

namespace wbc{

CartesianAccelerationConstraint::CartesianAccelerationConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

CartesianAccelerationConstraint::~CartesianAccelerationConstraint(){
}

void CartesianAccelerationConstraint::setReference(const base::samples::CartesianState& ref){

    CartesianConstraint::setReference(ref);

    if(!ref.hasValidAcceleration()){
        LOG_ERROR("Constraint %s has invalid linear and/or angular acceleration", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    this->time = ref.time;
    this->y_ref.segment(0,3) = ref.acceleration.linear;
    this->y_ref.segment(3,3) = ref.acceleration.angular;
}

} // namespace wbc
