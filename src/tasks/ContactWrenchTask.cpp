#include "ContactWrenchTask.hpp"
#include "../types/Wrench.hpp"
#include <iostream>

namespace wbc {

ContactWrenchTask::ContactWrenchTask(TaskConfig config,
                                     RobotModelPtr robot_model) :
    Task(config, robot_model, 6, TaskType::contact_wrench){

}

void ContactWrenchTask::update(){

    // Task Jacobian is identity here: The external wrenches are explicit variables in the QP,
    // so the reference wrench is simply forwarded to these variables
    A = Eigen::MatrixXd::Identity(nv,nv);
    Aw = Eigen::MatrixXd::Identity(nv,nv);
}

void ContactWrenchTask::setReference(const types::Wrench& ref){
    this->y_ref.segment(0,3) = ref.force;
    this->y_ref.segment(3,3) = ref.torque;
}


}
