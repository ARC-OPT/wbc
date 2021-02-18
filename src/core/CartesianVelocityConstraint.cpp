#include "CartesianVelocityConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

CartesianVelocityConstraint::CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

CartesianVelocityConstraint::~CartesianVelocityConstraint(){
}

void CartesianVelocityConstraint::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!ref.hasValidTwist()){
        LOG_ERROR("Constraint %s has invalid velocity and/or angular velocity", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;
    this->y_ref.segment(0,3) = ref.twist.linear;
    this->y_ref.segment(3,3) = ref.twist.angular;
}

} // namespace wbc
