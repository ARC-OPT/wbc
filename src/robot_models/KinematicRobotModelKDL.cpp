#include "KinematicRobotModelKDL.hpp"
#include "../common/TaskFrameKDL.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "RobotModelConfig.hpp"

namespace wbc{

KinematicRobotModelKDL::KinematicRobotModelKDL(const std::string& _base_frame) :
    RobotModel(_base_frame){

}

bool KinematicRobotModelKDL::loadModel(const RobotModelConfig& config){

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(config.file, tree)){
        LOG_ERROR("Unable to load KDL tree from file %s", config.file.c_str());
        return false;
    }

    if(full_tree.getNrOfSegments() == 0){
        full_tree = tree;

        // If no base frame is given, choose root of first KDL model as base
        if(base_frame.empty())
            base_frame = full_tree.getRootSegment()->first;
    }
    else{
        if(!hasFrame(config.hook)){
            LOG_ERROR("Trying to hook a new tree to frame %s, but this frame does not exist", config.hook.c_str());
            return false;
        }

        KDL::Frame initial_pose_kdl = KDL::Frame::Identity();
        std::string root = tree.getRootSegment()->first;

        if(config.initial_pose.hasValidPosition() && config.initial_pose.hasValidOrientation())
            kdl_conversions::RigidBodyState2KDL(config.initial_pose, initial_pose_kdl);
        else{
            LOG_ERROR("Trying to add urdf model with root %s at hook %s, but ist initial pose is invalid!", root.c_str(), config.hook.c_str());
            return false;
        }

        if(!full_tree.addSegment(KDL::Segment(root, KDL::Joint(KDL::Joint::None), initial_pose_kdl), config.hook)){
            LOG_ERROR("Unable to hook segment %s to segment %s", root.c_str(), config.hook.c_str());
            return false;
        }
        if(!full_tree.addTree(tree, root)){
            LOG_ERROR("Unable to add KDL tree at segment %s", root.c_str(), config.hook.c_str());
            return false;
        }
    }

    LOG_DEBUG("Successfully added tree with root '%s' to hook '%s'", tree.getRootSegment()->second.segment.getName().c_str(), config.hook.c_str());

    return true;
}


void KinematicRobotModelKDL::update(const base::samples::Joints &joint_state){
    for(size_t i = 0; i < task_frames.size(); i++)
        ((TaskFrameKDL*)task_frames[i])->updateTaskFrame(joint_state, joint_names);
}

void KinematicRobotModelKDL::update(const base::samples::RigidBodyState &new_pose){
    for(size_t i = 0; i < task_frames.size(); i++)
        ((TaskFrameKDL*)task_frames[i])->updateSegment(new_pose);
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().find(name) != full_tree.getSegments().end();
}

bool KinematicRobotModelKDL::addTaskFrameInternal(const std::string &tf_name){

    KDL::Chain chain;
    if(!full_tree.getChain(base_frame, tf_name, chain))
    {
        LOG_ERROR("Could not extract kinematic chain between %s and %s from tree", base_frame.c_str(), tf_name.c_str());
        return false;
    }

    TaskFrameKDL* tf = new TaskFrameKDL(chain, tf_name);
    tf->pose.targetFrame = base_frame;

    LOG_DEBUG("Sucessfully added task frame %s", tf_name.c_str());
    LOG_DEBUG("TF Vector now contains:");
    for(size_t i = 0; i < task_frames.size(); i++)
        LOG_DEBUG("%s", task_frames[i].name.c_str());
    LOG_DEBUG("\n");

    return true;
}

}
