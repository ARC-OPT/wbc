#include "ContactForceTask.hpp"
#include "../types/Wrench.hpp"
#include <iostream>

namespace wbc {

ContactForceTask::ContactForceTask(TaskConfig config,
                                     RobotModelPtr robot_model) :
    Task(config, robot_model, 3, TaskType::contact_force){
}

void ContactForceTask::update(){

    // Task Jacobian is identity here: The external wrenches are explicit variables in the QP,
    // so the reference wrench is simply forwarded to these variables
    A = Eigen::MatrixXd::Identity(nv,nv);
    Aw = Eigen::MatrixXd::Identity(nv,nv);
}

void ContactForceTask::setReference(const types::Wrench& ref){
    this->y_ref = ref.force;
}


}
