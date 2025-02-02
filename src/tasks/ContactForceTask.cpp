#include "ContactForceTask.hpp"
#include "../types/Wrench.hpp"
#include <iostream>

namespace wbc {

ContactForceTask::ContactForceTask(TaskConfig config,
                                     RobotModelPtr robot_model,
                                     const std::string &ref_frame) :
    Task(config, robot_model, 3, TaskType::contact_force), ref_frame(ref_frame){

}

void ContactForceTask::update(){

    // Task Jacobian is identity here: The external wrenches are explicit variables in the QP,
    // so the reference wrench is simply forwarded to these variables
    A = Eigen::MatrixXd::Identity(nv,nv);
    Aw = Eigen::MatrixXd::Identity(nv,nv);

    if(robot_model->worldFrame() != ref_frame){
        ref_frame_rotation = robot_model->pose(ref_frame).orientation.toRotationMatrix();

        // Transform reference to world frame
        y_ref_world = ref_frame_rotation * y_ref;

        // Also transform the weight vector from ref frame to the world frame. Take the absolute values after rotation, since weights can only
        // assume positive values
        weights_world.segment(0,3) = ref_frame_rotation * weights;
        weights_world = weights_world.cwiseAbs();
    }
    else{
        y_ref_world = y_ref;
        weights_world = weights;
    }
}

void ContactForceTask::setReference(const types::Wrench& ref){
    this->y_ref = ref.force;
}


}
