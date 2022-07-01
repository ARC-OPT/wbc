#include "CoMVelocityTask.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc {

CoMVelocityTask::CoMVelocityTask(TaskConfig config, uint n_robot_joints)
    : CartesianTask(config, n_robot_joints){
}

void CoMVelocityTask::update(RobotModelPtr robot_model){
    A = robot_model->comJacobian();
    // CoM tasks are always in world/base frame, no need to transform.
    y_ref_root = y_ref;
    weights_root = weights;
}

void CoMVelocityTask::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!base::isnotnan(ref.twist.linear)){
        LOG_ERROR("Constraint %s has invalid linear velocity", config.name.c_str())
        throw std::invalid_argument("Invalid constraint reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;
    this->y_ref.segment(0,3) = ref.twist.linear;
}

}
