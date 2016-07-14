#include "KinematicRobotModelKDL.hpp"
#include "../common/TaskFrameKDL.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "RobotModelConfig.hpp"
#include <fstream>

namespace wbc{

KinematicRobotModelKDL::KinematicRobotModelKDL(){

}


bool KinematicRobotModelKDL::configure(const std::vector<RobotModelConfig>& _robot_model_config,
                                       const std::vector<std::string>& _task_frame_ids,
                                       const std::string &_base_frame){

    clear();

    robot_model_config = _robot_model_config;

    for(size_t i = 0; i < robot_model_config.size(); i++){
        if(!loadURDFModel(robot_model_config[i]))
            return false;
    }

    for(size_t i = 0; i < _task_frame_ids.size(); i++)
        if(!addTaskFrame(_task_frame_ids[i]))
            return false;

    base_frame = _base_frame;

    // Add all joint names of all task frames to the joint names vector
    for(size_t i = 0; i < task_frames.size(); i++){

        TaskFrame* tf = task_frames[i];

        for(size_t j = 0; j < tf->joint_names.size(); j++){

            // Only add joint name if it is not yet in joint name vector
            if (std::find(joint_names.begin(), joint_names.end(), tf->joint_names[j]) == joint_names.end())
                joint_names.push_back(tf->joint_names[j]);
        }
    }

    return true;
}

void KinematicRobotModelKDL::update(const base::samples::Joints &joint_state,
                                    const std::vector<base::samples::RigidBodyState>& poses){

    current_joint_state = joint_state;

    for(size_t i = 0; i < task_frames.size(); i++)
        ((TaskFrameKDL*)task_frames[i])->update(joint_state, poses, joint_names);
}

void KinematicRobotModelKDL::getState(const std::string& tf_one_name,
                                      const std::string& tf_two_name,
                                      base::samples::RigidBodyState& state){

    TaskFrameKDL* tf_one = (TaskFrameKDL*)getTaskFrame(tf_one_name);
    TaskFrameKDL* tf_two = (TaskFrameKDL*)getTaskFrame(tf_two_name);

    state.sourceFrame = tf_two_name;
    state.targetFrame = tf_one_name;
    state.time = tf_one->pose.time > tf_two->pose.time ? tf_one->pose.time : tf_two->pose.time; // Use the latest time of the two task frames
    state.setTransform(tf_one->pose.getTransform().inverse() * tf_two->pose.getTransform());    // Pose of tf two in tf one coordinates
}

void KinematicRobotModelKDL::getState(const std::vector<std::string> &joint_names,
                                      base::samples::Joints& state){

    state.resize(joint_names.size());
    state.names = joint_names;

    for(uint i = 0; i < joint_names.size(); i++){
        uint idx;
        try{
           idx = current_joint_state.mapNameToIndex(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Requesting state of joint %s but this joint is not in joint state vector of robot model",
                      joint_names[i].c_str());
            throw std::invalid_argument("Invalid joint name");
        }
        state[i] = current_joint_state[idx];
    }
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().find(name) != full_tree.getSegments().end();
}

void KinematicRobotModelKDL::clear(){
    robot_model_config.clear();
    current_joint_state.clear();
    joint_index_map.clear();
    joint_names.clear();
    full_tree = KDL::Tree();
    base_frame = "";
    for(size_t i = 0; i < task_frames.size(); i++)
        delete task_frames[i];
    task_frames.clear();
}

bool KinematicRobotModelKDL::loadURDFModel(const RobotModelConfig& config){

    std::ifstream file_stream(config.file.c_str());
    if(!file_stream.is_open()){
        LOG_ERROR("Unable to open file %s. Maybe the file does not exist?", config.file.c_str());
        return false;
    }
    std::string urdf_string((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());

    KDL::Tree tree;
    if(!kdl_parser::treeFromString(urdf_string, tree)){
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
            if(config.hook.empty()){
                LOG_ERROR("Empty hook member of RobotModelConfig. Is this intended?");
                return false;
            }
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

    LOG_INFO("Successfully added tree with root '%s' to hook '%s'", tree.getRootSegment()->second.segment.getName().c_str(), config.hook.c_str());

    return true;
}

bool KinematicRobotModelKDL::addTaskFrame(const std::string &tf_name){

    if(!hasTaskFrame(tf_name)){

        KDL::Chain chain;
        if(!full_tree.getChain(base_frame, tf_name, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from tree", base_frame.c_str(), tf_name.c_str());
            return false;
        }

        TaskFrameKDL* tf = new TaskFrameKDL(chain, tf_name);
        tf->pose.targetFrame = base_frame;
        tf->pose.sourceFrame = tf_name;

        task_frames.push_back(tf);

        LOG_INFO("Sucessfully added task frame %s", tf_name.c_str());
        LOG_INFO("TF Vector now contains:");
        for(size_t i = 0; i < task_frames.size(); i++)
            LOG_INFO("%s", task_frames[i]->name.c_str());
        LOG_INFO("\n");
    }
    else
        LOG_WARN("Task Frame with ID %s has already been added", tf_name.c_str());

    return true;
}

}
