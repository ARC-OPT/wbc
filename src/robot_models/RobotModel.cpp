#include "RobotModel.hpp"
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>
#include "../common/TaskFrame.hpp"

namespace wbc{

RobotModel::RobotModel(){
}

RobotModel::~RobotModel(){
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
