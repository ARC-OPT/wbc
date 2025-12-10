#include "SpatialVelocityTask.hpp"

namespace wbc{

SpatialVelocityTask::SpatialVelocityTask(TaskConfig config,
                                             RobotModelPtr robot_model,
                                             const std::string &tip_frame)
    : Task(config, robot_model, 6, TaskType::spatial_velocity), tip_frame(tip_frame){
}

void SpatialVelocityTask::update(){
    
    // Task Jacobian
    A = robot_model->spaceJacobian(tip_frame);
}

void SpatialVelocityTask::setReference(const types::Twist& ref){
    this->y_ref.segment(0,3) = ref.linear;
    this->y_ref.segment(3,3) = ref.angular;
}

} // namespace wbc
