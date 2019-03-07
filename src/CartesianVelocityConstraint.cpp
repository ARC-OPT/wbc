#include "CartesianVelocityConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <wbc_common/CartesianState.hpp>

namespace wbc{

CartesianVelocityConstraint::CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){

    uint n_vars = config.nVariables();

    jacobian.setZero(6,n_vars);
    H.setZero(n_vars,6);
    Uf.resize(6, n_vars);
    Uf.setIdentity();
    Vf.resize(n_vars, n_vars);
    Vf.setIdentity();
    Sf.setZero(n_vars);
    tmp.setZero(6);
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
