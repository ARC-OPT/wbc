#include "CoMAccelerationConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc {

CoMAccelerationConstraint::CoMAccelerationConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

CoMAccelerationConstraint::~CoMAccelerationConstraint(){
}

void CoMAccelerationConstraint::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!base::isnotnan(ref.acceleration.linear)){
        LOG_ERROR("Constraint %s has invalid linear acceleration", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;
    this->y_ref.segment(0,3) = ref.acceleration.linear;
}

}
