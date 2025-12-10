#include "JointAccelerationTask.hpp"

namespace wbc{

JointAccelerationTask::JointAccelerationTask(TaskConfig config,
                                             RobotModelPtr robot_model,
                                             const std::vector<std::string>& joint_names)
    : Task(config, robot_model, robot_model->na(), TaskType::joint_acceleration), joint_names(joint_names){

}

void JointAccelerationTask::update(){

    // Joint space tasks: task matrix has only ones and Zeros. The joint order in the tasks might be different than in the robot model.
    // Thus, for joint space tasks, the joint indices have to be mapped correctly.
    A.setZero();
    A.block(0,robot_model->nfb(),nv,nv).setIdentity(); // Set identity for the actuated joints
}

void JointAccelerationTask::setReference(const Eigen::VectorXd& ref){
    assert(ref.size() == nv);
    this->y_ref = ref;
}
} // namespace wbc
