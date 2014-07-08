#include "RobotModelKDL.hpp"
#include <base/Logging.hpp>

namespace wbc{

RobotModelKDL::RobotModelKDL(const KDL::Tree& tree, const std::vector<std::string>& joint_names){
    tree_ = tree;

    // Create joint index map:

    // The joint name parameter can define the internal order of joints.
    // If no joint names are given, the order will be the same as in the KDL tree
    if(joint_names.empty())
    {
        KDL::SegmentMap segments = tree_.getSegments();
        uint idx = 0;
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
                joint_index_map_[seg.getJoint().getName()] = idx++;
        }
    }
    else
    {
        for(uint i = 0; i < joint_names.size(); i++)
            joint_index_map_[joint_names[i]] = i;

        //Check if all joints in tree are in joint index map
        KDL::SegmentMap segments = tree_.getSegments();
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
            {
                if(joint_index_map_.count(seg.getJoint().getName()) == 0)
                {
                    LOG_ERROR("Joint with name %s is in KDL::Tree but not in joint names parameter", seg.getJoint().getName().c_str());
                    LOG_ERROR("If the order of joints shall be fixed with the joint names parameter, all joints in tree have to be given here");
                    throw std::invalid_argument("Invalid joint names");
                }
            }
        }
    }
}

RobotModelKDL::~RobotModelKDL(){
    for(TaskFrameKDLMap::iterator it = tf_kdl_map_.begin(); it != tf_kdl_map_.end(); it++)
        delete it->second;
}

bool RobotModelKDL::addTaskFrame(const std::string &id){
    if(tf_kdl_map_.count(id) == 0)
    {
        KDL::Chain chain;
        std::string robot_root = tree_.getRootSegment()->first;
        if(!tree_.getChain(robot_root, id, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root.c_str(), id.c_str());
            return false;
        }
        TaskFrameKDL* tf_kdl = new TaskFrameKDL(chain, joint_index_map_);
        tf_kdl_map_[id] = tf_kdl;

        LOG_DEBUG("Sucessfully added task frame %s", id.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(TaskFrameKDLMap::iterator it = tf_kdl_map_.begin(); it != tf_kdl_map_.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_WARN("Task Frame with id %s has already been added", id.c_str());

    return true;
}

void RobotModelKDL::update(const base::samples::Joints &joint_state){
    for(TaskFrameKDLMap::iterator it = tf_kdl_map_.begin(); it != tf_kdl_map_.end(); it++)
        it->second->update(joint_state);
}

TaskFrameKDL* RobotModelKDL::getTaskFrame(const std::string &id){
    if(tf_kdl_map_.count(id) == 0){
        LOG_ERROR("Task Frame %s does not exist in Robot model", id.c_str());
        throw std::invalid_argument("Invalid task frame id");
    }

    return tf_kdl_map_[id];
}

}
