#include "SpatialAccelerationTask.hpp"
#include "../tools/Logger.hpp"

namespace wbc{

SpatialAccelerationTask::SpatialAccelerationTask(TaskConfig config,
                                                     RobotModelPtr robot_model,
                                                     const std::string &tip_frame)
    : Task(config, robot_model, 6, TaskType::spatial_acceleration), tip_frame(tip_frame){
}

void SpatialAccelerationTask::update(){
    // Task Jacobian
    A = robot_model->spaceJacobian(tip_frame);

    // Desired task space acceleration: y_r = y_d - Jdot*qdot
    y_ref.segment(0,3) -= robot_model->spatialAccelerationBias(tip_frame).linear;
    y_ref.segment(3,3) -= robot_model->spatialAccelerationBias(tip_frame).angular;
}

void SpatialAccelerationTask::setReference(const types::SpatialAcceleration& ref){
    this->y_ref.segment(0,3) = ref.linear;
    this->y_ref.segment(3,3) = ref.angular;
}

} // namespace wbc
