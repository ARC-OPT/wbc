#include "RobotModel.hpp"
#include "TaskFrame.hpp"
#include <base/logging.h>

RobotModel::RobotModel(const KDL::Tree tree){
    tree_ = tree;
    no_of_joints_ = tree_.getNrOfJoints();

    LOG_DEBUG("Created Robot Model. No of joints is %i\n", no_of_joints_);
}

RobotModel::~RobotModel(){
    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
        delete it->second;
    task_frame_map_.clear();
}

bool RobotModel::addTaskFrame(const std::string &frame_id){
    if(task_frame_map_.count(frame_id) == 0){
        std::string robot_root = tree_.getRootSegment()->first;
        KDL::Chain chain;
        if(!tree_.getChain(robot_root, frame_id, chain)){
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root.c_str(), frame_id.c_str());
            return false;
        }
        TaskFrame* tf = new TaskFrame(chain, no_of_joints_);
        task_frame_map_[frame_id] = tf;

        LOG_DEBUG("Sucessfully added task frame %s", frame_id.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_INFO("Task Frame with id %s has already been added", frame_id.c_str());

    return true;
}

TaskFrame* RobotModel::getTaskFrame(const std::string &frame_id){
    if(task_frame_map_.count(frame_id) == 0)
        return 0;
    else
        return task_frame_map_[frame_id];
}

void RobotModel::update(const base::samples::Joints &status){

    //if not done yet create joint index vector
    if(joint_index_map_.empty()){
        for(uint i = 0; i < status.size(); i++)
            joint_index_map_[status.names[i]] = i;
    }

    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++){
        TaskFrame* tf = it->second;
        tf->update(status);
    }
}
