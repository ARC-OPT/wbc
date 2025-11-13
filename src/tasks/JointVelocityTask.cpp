#include "JointVelocityTask.hpp"

namespace wbc{

JointVelocityTask::JointVelocityTask(TaskConfig config,
                                     RobotModelPtr robot_model,
                                     const std::vector<std::string>& joint_names)
    : Task(config, robot_model, joint_names.size(), TaskType::joint_velocity), joint_names(joint_names){

}

void JointVelocityTask::update(){

    // Joint space tasks: task matrix has only ones and Zeros. The joint order in the tasks might be different than in the robot model.
    // Thus, for joint space tasks, the joint indices have to be mapped correctly.
    A.setZero();
    y_ref_world.setZero();
    weights_world.setZero();
    A.block(0,robot_model->nfb(),nv,nv).setIdentity(); // Set identity for the actuated joints
    
    y_ref_world.tail(y_ref.size()) = y_ref;     // In joint space y_ref is equal to y_ref_root
    weights_world.tail(y_ref.size()) = weights; // Same for the weights
}

void JointVelocityTask::setReference(const Eigen::VectorXd &ref){
    assert(ref.size() == nv);
    this->y_ref = ref;
}
} // namespace wbc
