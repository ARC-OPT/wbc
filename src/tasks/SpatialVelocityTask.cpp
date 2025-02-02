#include "SpatialVelocityTask.hpp"

namespace wbc{

SpatialVelocityTask::SpatialVelocityTask(TaskConfig config,
                                             RobotModelPtr robot_model,
                                             const std::string &tip_frame,
                                             const std::string &ref_frame)
    : Task(config, robot_model, 6, TaskType::spatial_velocity), tip_frame(tip_frame), ref_frame(ref_frame){
}

void SpatialVelocityTask::update(){
    
    // Task Jacobian
    A = robot_model->spaceJacobian(tip_frame);

    // Convert reference twist to world frame. We transform only the orientation of the
    // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
    // be the origin of ref frame, not the root frame.
    if(robot_model->worldFrame() != ref_frame){
        rot_mat = robot_model->pose(ref_frame).orientation.toRotationMatrix();
        y_ref_world.segment(0,3) = rot_mat * y_ref.segment(0,3);
        y_ref_world.segment(3,3) = rot_mat * y_ref.segment(3,3);
    }
    else
        y_ref_world = y_ref;


    // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
    // assume positive values
    if(robot_model->worldFrame() != ref_frame){
        weights_world.segment(0,3) = rot_mat * weights.segment(0,3);
        weights_world.segment(3,3) = rot_mat * weights.segment(3,3);
        weights_world = weights_world.cwiseAbs();
    }
    else
        weights_world = weights;
}

void SpatialVelocityTask::setReference(const types::Twist& ref){
    this->y_ref.segment(0,3) = ref.linear;
    this->y_ref.segment(3,3) = ref.angular;
}

} // namespace wbc
