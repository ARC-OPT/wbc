#include "KinematicRobotModelKDL.hpp"
#include "TaskFrameKDL.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace wbc{

bool KinematicRobotModelKDL::addModelFromFile(const std::string& model_file,
                                              base::samples::RigidBodyState& initial_pose,
                                              const std::string hook_name){

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(model_file, tree)){
        LOG_ERROR("Unable to load KDL tree from file %s", model_file.c_str());
        return false;
    }

    KDL::Frame initial_pose_kdl;
    kdl_conversions::RigidBodyState2KDL(initial_pose, initial_pose_kdl);

    if(full_tree.getNrOfSegments() == 0)
        full_tree = tree;
    else{
        if(!hasFrame(hook_name)){
            LOG_ERROR("Trying to hook a new tree to frame %s, but this frame does not exist", hook_name.c_str());
            return false;
        }

        KDL::Frame initial_pose_kdl = KDL::Frame::Identity();
        std::string root = tree.getRootSegment()->first;
        if(initial_pose.hasValidPosition() && initial_pose.hasValidOrientation())
            kdl_conversions::RigidBodyState2KDL(initial_pose, initial_pose_kdl);
        else{
            LOG_ERROR("Trying to add urdf model with root %s at hook %s, but ist initial pose is invalid!", root.c_str(), hook_name.c_str());
            return false;
        }

        if(!full_tree.addSegment(KDL::Segment(root, KDL::Joint(KDL::Joint::None), initial_pose_kdl), hook_name)){
            LOG_ERROR("Unable to hook segment %s to segment %s", root.c_str(), hook_name.c_str());
            return false;
        }
        if(!full_tree.addTree(tree, root)){
            LOG_ERROR("Unable to add KDL tree at segment %s", root.c_str(), hook_name.c_str());
            return false;
        }
    }

    LOG_DEBUG("Successfully added tree with root '%s' to hook '%s'", tree.getRootSegment()->second.segment.getName().c_str(), hook_name.c_str());

    return true;
}

bool KinematicRobotModelKDL::addTaskFrame(const std::string &tf_name){

    if(!hasTaskFrame(tf_name))
    {
        KDL::Chain chain;
        const std::string &root_name = full_tree.getRootSegment()->first;
        if(!full_tree.getChain(root_name, tf_name, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from tree", root_name.c_str(), tf_name.c_str());
            return false;
        }

        task_frames.push_back(new TaskFrameKDL(chain, tf_name));

        LOG_DEBUG("Sucessfully added task frame %s", tf_name.c_str());
        LOG_DEBUG("TF Vector now contains:");
        for(size_t i = 0; i < task_frames.size(); i++)
            LOG_DEBUG("%s", task_frames[i].name.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_WARN("KinematicRobotModelKDL::addTaskFrame: Task Frame with id %s has already been added", tf_name.c_str());

    return true;
}

void KinematicRobotModelKDL::removeTaskFrame(const std::string &tf_name){
    for(size_t i = 0; i < task_frames.size(); i++){
        if(task_frames[i]->name == tf_name){
            task_frames.erase(task_frames.begin() + i);
            return;
        }
    }
    LOG_WARN("KinematicRobotModelKDL::removeTaskFrame: Task Frame with id %s does not exist", tf_name.c_str());
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().find(name) != full_tree.getSegments().end();
}

}
