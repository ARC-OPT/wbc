#include "CartesianVelocityConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

CartesianVelocityConstraint::CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : CartesianConstraint(config, n_robot_joints){
}

void CartesianVelocityConstraint::update(RobotModelPtr robot_model){
    
    // Constraint Jacobian
    A = robot_model->spaceJacobian(config.root, config.tip);

    // Convert constraint twist to robot root
    base::MatrixXd rot_mat = robot_model->rigidBodyState(config.root, config.ref_frame).pose.orientation.toRotationMatrix();
    y_ref_root.segment(0,3) = rot_mat * y_ref.segment(0,3);
    y_ref_root.segment(3,3) = rot_mat * y_ref.segment(3,3);

    // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
    // assume positive values
    weights_root.segment(0,3) = rot_mat * weights.segment(0,3);
    weights_root.segment(3,3) = rot_mat * weights.segment(3,3);
    weights_root = weights_root.cwiseAbs();
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
