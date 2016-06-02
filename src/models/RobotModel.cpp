#include "RobotModel.hpp"
#include <base/Logging.hpp>

namespace wbc{

RobotModel::RobotModel(){
}

RobotModel::~RobotModel(){
}

bool RobotModel::addTaskFrames(const std::vector<std::string> &task_frame_ids){
    for(size_t i = 0; i < task_frame_ids.size(); i++)
        if(!addTaskFrame(task_frame_ids[i]))
            return false;
    return true;
}

void RobotModel::update(const base::samples::Joints &joint_state){
    for(size_t i = 0; i < task_frames.size(); i++)
        task_frames[i]->update(joint_state);
}

void RobotModel::update(const base::samples::RigidBodyState &new_pose){
    for(size_t i = 0; i < task_frames.size(); i++)
        task_frames[i]->update(new_pose);
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
