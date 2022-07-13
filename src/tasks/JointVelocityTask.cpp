#include "JointVelocityTask.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

JointVelocityTask::JointVelocityTask(TaskConfig config, uint n_robot_joints)
    : JointTask(config, n_robot_joints){

}

void JointVelocityTask::update(RobotModelPtr robot_model){

    // Joint space tasks: task matrix has only ones and Zeros. The joint order in the tasks might be different than in the robot model.
    // Thus, for joint space tasks, the joint indices have to be mapped correctly.
    for(uint k = 0; k < config.joint_names.size(); k++){

        int idx = robot_model->jointIndex(config.joint_names[k]);
        A(k,idx) = 1.0;
        y_ref_root = y_ref;     // In joint space y_ref is equal to y_ref_root
        weights_root = weights; // Same of the weights
    }
}

void JointVelocityTask::setReference(const base::commands::Joints& ref){

    if(ref.size() != config.nVariables()){
        LOG_ERROR("Task %s: Size of reference input is %i, but should be %i", config.name.c_str(), ref.size(), config.nVariables());
        throw std::invalid_argument("Invalid task reference input");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;

    for(size_t i = 0; i < ref.size(); i++){
        uint idx;
        try{
            idx = ref.mapNameToIndex(config.joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Task %s expects joint %s  but this joint is not in reference vector", config.name.c_str(), config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid task reference input");
        }

        if(!ref[idx].hasSpeed()){
            LOG_ERROR("Task %s: Reference input for joint %s has no valid speed value", config.name.c_str(), config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid task reference input");
        }

        this->y_ref(i) = ref[idx].speed;
    }
}
} // namespace wbc
