#include "CoMAccelerationTask.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc {

CoMAccelerationTask::CoMAccelerationTask(TaskConfig config, uint n_robot_joints)
    : CartesianTask(config, n_robot_joints){
}

void CoMAccelerationTask::update(RobotModelPtr robot_model){
    A = robot_model->comJacobian();
    // Desired task space acceleration: y_r = y_d - Jdot*qdot
    y_ref = y_ref - robot_model->spatialAccelerationBias(robot_model->worldFrame(), robot_model->baseFrame()).linear;
    // CoM tasks are always in world/base frame, no need to transform.
    y_ref_root = y_ref;
    weights_root = weights;
}

void CoMAccelerationTask::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!base::isnotnan(ref.acceleration.linear)){
        LOG_ERROR("Task %s has invalid linear acceleration", config.name.c_str())
        throw std::invalid_argument("Invalid task reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;
    this->y_ref.segment(0,3) = ref.acceleration.linear;
}

}
