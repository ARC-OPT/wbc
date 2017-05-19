#include "CartesianVelocityConstraint.hpp"

namespace wbc{

CartesianVelocityConstraint::CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : Constraint(config, n_robot_joints){

    uint n_vars = config.noOfConstraintVariables();

    jacobian.setZero(6,n_robot_joints);
    H.setZero(n_vars,6);
    Uf.resize(6, n_vars);
    Uf.setIdentity();
    Vf.resize(n_vars, n_vars);
    Vf.setIdentity();
    Sf.setZero(n_vars);
    tmp.setZero(6);
}

void CartesianVelocityConstraint::setReference(const base::samples::RigidBodyState& ref){

    if(!ref.hasValidVelocity() ||
       !ref.hasValidAngularVelocity())
        throw std::invalid_argument("Constraint " + config.name + " has invalid velocity and/or angular velocity");

    this->time = ref.time;
    this->y_ref.segment(0,3) = ref.velocity;
    this->y_ref.segment(3,3) = ref.angular_velocity;
}

} // namespace wbc
