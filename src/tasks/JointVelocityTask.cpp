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
    for(uint k = 0; k < joint_names.size(); k++){
        int idx = robot_model->jointIndex(joint_names[k]);
        A(k,idx) = 1.0;
    }
    y_ref_world = y_ref;     // In joint space y_ref is equal to y_ref_root
    weights_world = weights; // Same of the weights
}

void JointVelocityTask::setReference(const Eigen::VectorXd &ref){
    assert(ref.size() == nv);
    this->y_ref = ref;
}
} // namespace wbc
