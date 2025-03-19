#include "CoMAccelerationTask.hpp"

namespace wbc {

CoMAccelerationTask::CoMAccelerationTask(TaskConfig config,
                                         RobotModelPtr robot_model)
    : Task(config, robot_model, 3, TaskType::com_acceleration){
}

void CoMAccelerationTask::update(){
    A = robot_model->comJacobian();
    // Desired task space acceleration: y_r = y_d - Jdot*qdot
    y_ref = y_ref - robot_model->spatialAccelerationBias(robot_model->baseFrame()).linear;
    // CoM tasks are always in world frame, no need to transform.
    y_ref_world = y_ref;
    weights_world = weights;
}

void CoMAccelerationTask::setReference(const Eigen::Vector3d& ref){
    this->y_ref = ref;
}

}
