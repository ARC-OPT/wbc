#include "RobotModelKDL.hpp"
#include <base/Logging.hpp>

namespace wbc{

RobotModelKDL::RobotModelKDL(const KDL::Tree& tree){
    tree_ = tree;
}

RobotModelKDL::~RobotModelKDL(){
    for(KinChainMap::iterator it = kin_chain_map_.begin(); it != kin_chain_map_.end(); it++)
        delete it->second;
}

bool RobotModelKDL::addTaskFrames(const std::vector<std::string>& task_frame_ids){
    for(uint i = 0; i < task_frame_ids.size(); i++){
        if(!addTaskFrame(task_frame_ids[i]))
            return false;
    }
    return true;
}

bool RobotModelKDL::addTaskFrame(const std::string &id){
    if(kin_chain_map_.count(id) == 0)
    {
        KDL::Chain chain;
        std::string robot_root = tree_.getRootSegment()->first;
        if(!tree_.getChain(robot_root, id, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root.c_str(), id.c_str());
            return false;
        }
        addKinChain(chain, id);

        LOG_DEBUG("Sucessfully added task frame %s", id.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(KinChainMap::iterator it = kin_chain_map_.begin(); it != kin_chain_map_.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_WARN("Task Frame with id %s has already been added", id.c_str());

    return true;
}

void RobotModelKDL::addKinChain(const KDL::Chain& chain, const std::string &id)
{
    KinematicChainKDL* tf_chain_kdl = new KinematicChainKDL(chain);
    kin_chain_map_[id] = tf_chain_kdl;
}

void RobotModelKDL::update(const base::samples::Joints &joint_state){
    for(KinChainMap::iterator it = kin_chain_map_.begin(); it != kin_chain_map_.end(); it++)
        it->second->update(joint_state);
}

TaskFrameKDL* RobotModelKDL::getTaskFrame(const std::string &id){
    if(kin_chain_map_.count(id) == 0){
        LOG_ERROR("Task Frame %s does not exist in Robot model", id.c_str());
        throw std::invalid_argument("Invalid task frame id");
    }

    return kin_chain_map_[id]->tf;
}

void RobotModelKDL::getTFVector(std::vector<TaskFrameKDL>& task_frames){

    if(task_frames.size() != tf_vector_.size())
        task_frames.resize(tf_vector_.size());

    for(uint i = 0; i < tf_vector_.size(); i++)
        task_frames[i] = *tf_vector_[i];
}

}
