#include "CoMVelocityTask.hpp"
#include "../types/RigidBodyState.hpp"

namespace wbc {

CoMVelocityTask::CoMVelocityTask(TaskConfig config,
                                 RobotModelPtr robot_model)
    : Task(config, robot_model, 3, TaskType::com_velocity){
}

void CoMVelocityTask::update(){
    A = robot_model->comJacobian();
}

void CoMVelocityTask::setReference(const Eigen::Vector3d& ref){
    this->y_ref.segment(0,3) = ref;
}

}
