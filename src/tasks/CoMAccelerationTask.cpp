#include "CoMAccelerationTask.hpp"

namespace wbc {

CoMAccelerationTask::CoMAccelerationTask(TaskConfig config,
                                         uint nj)
    : Task(config, 3, nj, TaskType::com_acceleration){
}

void CoMAccelerationTask::update(RobotModelPtr robot_model){
    A = robot_model->comJacobian();
    // Desired task space acceleration: y_r = y_d - Jdot*qdot
    y_ref = y_ref - robot_model->spatialAccelerationBias(robot_model->baseFrame()).linear;
    // CoM tasks are always in world frame, no need to transform.
    y_ref_world = y_ref;
    y_ref_world = weights;
}

void CoMAccelerationTask::setReference(const Eigen::Vector3d& ref){
    this->y_ref = ref;
}

}
