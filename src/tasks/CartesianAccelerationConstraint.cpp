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

void CartesianAccelerationConstraint::update(RobotModelPtr robot_model){
    // Task Jacobian
    A = robot_model->spaceJacobian(config.root, config.tip);

    // Desired task space acceleration: y_r = y_d - Jdot*qdot
    base::samples::Joints joint_state = robot_model->jointState(robot_model->jointNames());
    base::VectorXd q_dot(robot_model->noOfJoints());
    for(size_t j = 0; j < joint_state.size(); j++)
        q_dot(j) = joint_state[j].speed;
    y_ref = y_ref - robot_model->spatialAccelerationBias(config.root, config.tip);

    // Convert input acceleration from the reference frame of the constraint to the base frame of the robot. We transform only the orientation of the
    // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
    // be the origin of ref frame, not the root frame. This is more intuitive when controlling the orientation of e.g. a robot' s end effector.
    base::samples::RigidBodyStateSE3 ref_frame = robot_model->rigidBodyState(config.root, config.ref_frame);
    y_ref_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * y_ref.segment(0,3);
    y_ref_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * y_ref.segment(3,3);

    // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
    // assume positive values
    weights_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * weights.segment(0,3);
    weights_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * weights.segment(3,3);
    weights_root = weights_root.cwiseAbs();
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
