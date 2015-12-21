#include "KinematicModel.hpp"
#include "TaskFrame.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>

namespace wbc{

bool KinematicModel::addTree(const KDL::Tree &tree,
                             const base::samples::RigidBodyState& initial_pose,
                             const std::string hook){

    if(full_tree.getNrOfSegments() == 0)
        full_tree = tree;
    else{
        if(!hasFrame(hook)){
            LOG_ERROR("Trying to hook a new tree to frame %s, but this frame does not exist", hook.c_str());
            return false;
        }

        KDL::Frame initial_pose_kdl = KDL::Frame::Identity();
        if(initial_pose.hasValidPosition() && initial_pose.hasValidOrientation())
            kdl_conversions::RigidBodyState2KDL(initial_pose, initial_pose_kdl);

        std::string root = tree.getRootSegment()->first;
        if(!full_tree.addSegment(KDL::Segment(root, KDL::Joint(KDL::Joint::None), initial_pose_kdl), hook)){
            LOG_ERROR("Unable to hook segment %s to segment %s", root.c_str(), hook.c_str());
            return false;
        }
        if(!full_tree.addTree(tree, root)){
            LOG_ERROR("Unable to add KDL tree at segment %s", root.c_str(), hook.c_str());
            return false;
        }
    }

    LOG_DEBUG("Successfully added tree with root '%s' to hook '%s'", tree.getRootSegment()->second.segment.getName().c_str(), hook.c_str());

    return true;
}

bool KinematicModel::addTaskFrame(const std::string &tf_name){

    if(tf_map.count(tf_name) == 0)
    {
        KDL::Chain chain;
        const std::string &root_name = full_tree.getRootSegment()->first;
        if(!full_tree.getChain(root_name, tf_name, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from tree", root_name.c_str(), tf_name.c_str());
            return false;
        }

        tf_map[tf_name] = TaskFrame(chain);

        LOG_DEBUG("Sucessfully added task frame %s", tf_name.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(TaskFrameMap::iterator it = tf_map.begin(); it != tf_map.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_WARN("KinematicModel::addTaskFrame: Task Frame with id %s has already been added", tf_name.c_str());

    return true;
}

bool KinematicModel::addTaskFrames(const std::vector<std::string> &task_frame_ids){
    for(size_t i = 0; i < task_frame_ids.size(); i++)
        if(!addTaskFrame(task_frame_ids[i]))
            return false;
    return true;
}

void KinematicModel::removeTaskFrame(const std::string &tf_name){
    if(tf_map.count(tf_name) > 0)
        tf_map.erase(tf_name);
    else
        LOG_WARN("KinematicModel::removeTaskFrame: Task Frame with id %s does not exist", tf_name.c_str());
}

void KinematicModel::updateJoints(const base::samples::Joints &joint_state){

    for(TaskFrameMap::iterator it = tf_map.begin(); it != tf_map.end(); it++)
        it->second.updateJoints(joint_state);
}

void KinematicModel::updateLink(const base::samples::RigidBodyState &new_pose){

    if(hasFrame(new_pose.sourceFrame)){

        for(TaskFrameMap::iterator it = tf_map.begin(); it != tf_map.end(); it++)
            it->second.updateLink(new_pose);
    }
    else{
        LOG_ERROR("Trying to update pose of segment %s, but this segment does not exist in tree", new_pose.sourceFrame.c_str());
        throw std::invalid_argument("Invalid segment pose");
    }
}

const TaskFrame& KinematicModel::getTaskFrame(const std::string &tf_name){
    if(tf_map.count(tf_name) == 0){
        LOG_ERROR("No such task frame: %s", tf_name.c_str());
        throw std::invalid_argument("Invalid task frame id");
    }
    else
        return tf_map[tf_name];
}

bool KinematicModel::hasFrame(const std::string &name){
    return full_tree.getSegments().find(name) != full_tree.getSegments().end();
}

}
