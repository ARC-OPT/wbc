#include "CartesianAccelerationConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

base::Vector6d operator+(base::Vector6d a, base::Acceleration b){
    a.segment(0,3) += b.linear;
    a.segment(3,3) += b.angular;
    return a;
}

base::Vector6d operator-(base::Vector6d a, base::Acceleration b){
    a.segment(0,3) -= b.linear;
    a.segment(3,3) -= b.angular;
    return a;
}

CartesianAccelerationConstraint::CartesianAccelerationConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

CartesianAccelerationConstraint::~CartesianAccelerationConstraint(){
}

void CartesianAccelerationConstraint::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!ref.hasValidAcceleration()){
        LOG_ERROR("Constraint %s has invalid linear and/or angular acceleration", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;
    this->y_ref.segment(0,3) = ref.acceleration.linear;
    this->y_ref.segment(3,3) = ref.acceleration.angular;
}

} // namespace wbc
