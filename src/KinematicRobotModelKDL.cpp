#include "KinematicRobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <base-logging/Logging.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "RobotModelConfig.hpp"

namespace wbc{

KinematicRobotModelKDL::KinematicRobotModelKDL(){
}

KinematicRobotModelKDL::~KinematicRobotModelKDL(){
    current_joint_state.clear();
    current_poses.clear();
    full_tree = KDL::Tree();

    KinematicChainKDLMap::const_iterator it;
    for(it = kdl_chain_map.begin(); it != kdl_chain_map.end(); it++)
        delete it->second;
    kdl_chain_map.clear();
}

bool KinematicRobotModelKDL::configure(const std::vector<RobotModelConfig>& model_config,
                                       const std::vector<std::string> &joint_names,
                                       const std::string &base_frame){

    for(size_t i = 0; i < model_config.size(); i++){

        KDL::Tree tree;
        if(!kdl_parser::treeFromFile(model_config[i].file, tree)){
            LOG_ERROR("Unable to parse urdf model from file %s", model_config[i].file.c_str());
            return false;
        }

        if(!addTree(tree, model_config[i].hook, model_config[i].initial_pose))
            return false;
    }

    // If no joint names are given, take them from KDL Tree
    this->joint_names = joint_names;
    if(this->joint_names.empty())
        this->joint_names = jointNamesFromTree(full_tree);

    // If no base frame is given, take it from KDL tree
    this->base_frame = base_frame;
    if(this->base_frame.empty())
        full_tree.getRootSegment()->second.segment.getName();

    return true;
}

void KinematicRobotModelKDL::createChain(const std::string &root_frame, const std::string &tip_frame){
    KDL::Chain chain;
    if(!full_tree.getChain(root_frame, tip_frame, chain)){
        LOG_ERROR("Unable to extract kinematics chain from %s to %s from KDL tree", root_frame.c_str(), tip_frame.c_str());
        throw std::invalid_argument("Invalid robot model config");
    }

    KinematicChainKDL* kin_chain = new KinematicChainKDL(chain, root_frame, tip_frame);
    kin_chain->update(current_joint_state, current_poses);
    kdl_chain_map[root_frame + "_" + tip_frame] = kin_chain;

    jac_map[root_frame + "_" + tip_frame] = Jacobian(kin_chain->joint_names.size());
}

bool KinematicRobotModelKDL::addTree(const KDL::Tree& tree, const std::string& hook, const base::samples::RigidBodyState &pose){

    if(full_tree.getNrOfSegments() == 0)
        full_tree = tree;
    else{
        if(hook.empty()){
            LOG_ERROR("Unexpected empty hook name. To which segment do you want to attach the tree?");
            return false;
        }
        if(!hasFrame(hook)){
            LOG_ERROR("Hook name is %s, but this segment does not exist in tree", hook.c_str());
            return false;
        }

        std::string root = tree.getRootSegment()->first;
        KDL::Frame pose_kdl;
        kdl_conversions::RigidBodyState2KDL(pose, pose_kdl);
        if(!full_tree.addSegment(KDL::Segment(root, KDL::Joint(KDL::Joint::None), pose_kdl), hook))
            throw std::invalid_argument("Unable to attach segment " + root + " to existing tree segment " + hook.c_str());
        if(!full_tree.addTree(tree, root)){
            LOG_ERROR("Unable to attach tree with root segment %s", root.c_str());
            return false;
        }
    }
    return true;
}

void KinematicRobotModelKDL::update(const base::samples::Joints& joint_state,
                                    const std::vector<base::samples::RigidBodyState>& poses){

    current_joint_state = joint_state;
    current_poses = poses;

    KinematicChainKDLMap::const_iterator it;
    for(it = kdl_chain_map.begin(); it != kdl_chain_map.end(); it++)
        it->second->update(joint_state, poses);
}

const base::samples::RigidBodyState &KinematicRobotModelKDL::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    // Create chain if it does not exist
    if(kdl_chain_map.count(root_frame + "_" + tip_frame) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDL* kdl_chain = kdl_chain_map[root_frame + "_" + tip_frame];
    if(kdl_chain->last_update.isNull()){
        LOG_ERROR("You have to call update() with appropriate timestamps at least once before requesting kinematic information!");
        throw std::runtime_error("Invalid call to rigidBodyState()");
    }

    return kdl_chain->rigid_body_state;
}

const base::samples::Joints& KinematicRobotModelKDL::jointState(const std::vector<std::string> &joint_names){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("You have to call update() with appropriate timestamps at least once before requesting kinematic information!");
        throw std::runtime_error("Invalid call to jointState()");
    }

    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    joint_state.time = current_joint_state.time;

    for(uint i = 0; i < joint_names.size(); i++){
        try{
            joint_state[i] = current_joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("KinematicRobotModelKDL: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state;
}

const base::MatrixXd& KinematicRobotModelKDL::jacobian(const std::string &root_frame, const std::string &tip_frame){

    // Create chain if it does not exist
    if(kdl_chain_map.count(root_frame + "_" + tip_frame) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDL* kdl_chain = kdl_chain_map[root_frame + "_" + tip_frame];
    if(kdl_chain->last_update.isNull()){
        LOG_ERROR("You have to call update() with appropriate timestamps at least once before requesting kinematic information!");
        throw std::runtime_error("Invalid call to jacobian()");
    }

    jac_map[root_frame + "_" + tip_frame].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_map[root_frame + "_" + tip_frame].col(idx) = kdl_chain->jacobian.data.col(j);
    }
    return jac_map[root_frame + "_" + tip_frame];
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().count(name) > 0;
}

std::vector<std::string> KinematicRobotModelKDL::jointNamesFromTree(const KDL::Tree &tree) const{

    std::vector<std::string> joint_names;
    KDL::SegmentMap::const_iterator it;
    const KDL::SegmentMap& segments = tree.getSegments();
    for(it = segments.begin(); it!= segments.end(); it++) {
        KDL::Joint joint = it->second.segment.getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
    return joint_names;
}

}
