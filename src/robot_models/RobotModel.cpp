#include "RobotModel.hpp"
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>
#include "../common/TaskFrame.hpp"

namespace wbc{

RobotModel::RobotModel(const std::string& _base_frame) :
    base_frame(_base_frame){
}

RobotModel::~RobotModel(){
}

bool RobotModel::addTaskFrame(const std::string &tf_name){
    if(!hasTaskFrame(tf_name)){
        if(!addTaskFrameInternal(tf_name))
            return false;

        TaskFrame* tf = getTaskFrame(tf_name);

        // Add all joint names of the task frame to the joint names vector
        for(size_t i = 0; i < tf->joint_names.size(); i++){

            // Only add joint name if it is not yet in joint name vector
            if (std::find(joint_names.begin(), joint_names.end(), tf->joint_names[i]) == joint_names.end())
                joint_names.push_back(tf->joint_names[i]);
        }
    }
    else
        LOG_WARN("Task Frame with ID %s has already been added", tf_name.c_str());

    return true;
}

bool RobotModel::addTaskFrames(const std::vector<std::string> &tf_names){
    for(size_t i = 0; i < tf_names.size(); i++){
        if(!addTaskFrame(tf_names[i]))
            return false;
    }
    return true;
}

void RobotModel::removeTaskFrame(const std::string &tf_name){
    for(size_t i = 0; i < task_frames.size(); i++){
        if(task_frames[i]->name == tf_name){
            task_frames.erase(task_frames.begin() + i);
            return;
        }
    }
    LOG_WARN("RobotModel::removeTaskFrame: Task Frame with id %s does not exist", tf_name.c_str());
}

TaskFrame* RobotModel::getTaskFrame(const std::string &tf_name){
    for(size_t i = 0; i < task_frames.size(); i++){
        if(task_frames[i]->name == tf_name)
            return task_frames[i];
    }
    LOG_ERROR("No such task frame: %s", tf_name.c_str());
    throw std::invalid_argument("Invalid task frame id");
}

bool RobotModel::hasTaskFrame(const std::string &name){
    for(size_t i = 0; i < task_frames.size(); i++){
        if(task_frames[i]->name == name)
            return true;
    }
    return false;
}

}
