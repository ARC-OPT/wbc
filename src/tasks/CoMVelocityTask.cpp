#include "CoMVelocityTask.hpp"
#include "../types/RigidBodyState.hpp"

namespace wbc {

CoMVelocityTask::CoMVelocityTask(TaskConfig config,
                                 uint nj)
    : Task(config, 3, nj, TaskType::com_velocity){
}

void CoMVelocityTask::update(RobotModelPtr robot_model){
    A = robot_model->comJacobian();
    // CoM tasks are always in world/base frame, no need to transform.
    y_ref_world = y_ref;
    weights_world = weights;
}

void CoMVelocityTask::setReference(const Eigen::Vector3d& ref){
    this->y_ref.segment(0,3) = ref;
}

}
