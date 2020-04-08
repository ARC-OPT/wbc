#include "KinematicRobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../core/RobotModelConfig.hpp"

namespace wbc{

KinematicRobotModelKDL::KinematicRobotModelKDL(){
}

KinematicRobotModelKDL::~KinematicRobotModelKDL(){
}

void KinematicRobotModelKDL::clear(){
    current_joint_state.clear();
    full_tree = KDL::Tree();
    kdl_chain_map.clear();
    jac_map.clear();
    jac_dot_map.clear();
    robot_models_state.clear();
}


bool KinematicRobotModelKDL::configure(const std::string& model_filename,
                                       const std::vector<std::string> &joint_names,
                                       const std::string &base_frame){
    clear();

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(model_filename, tree)){
        LOG_ERROR("Unable to parse urdf model from file %s", model_filename.c_str());
        return false;
    }

    if(!addTree(tree))
        return false;

    // If no joint names are given, take them from KDL Tree
    this->joint_names = joint_names;
    if(this->joint_names.empty())
        this->joint_names = jointNamesFromTree(full_tree);

    // If no base frame is given, take it from KDL tree
    this->base_frame = base_frame;
    if(this->base_frame.empty())
        this->base_frame = full_tree.getRootSegment()->second.segment.getName();

    return true;
}

bool KinematicRobotModelKDL::configure(const std::vector<RobotModelConfig>& model_config,
                                       const std::vector<std::string> &joint_names,
                                       const std::string &base_frame){

    clear();

    for(const RobotModelConfig& cfg : model_config){

        KDL::Tree tree;
        if(!kdl_parser::treeFromFile(cfg.file, tree)){
            LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
            return false;
        }

        if(!addTree(tree, cfg.hook, cfg.initial_pose))
            return false;
    }

    // If no joint names are given, take them from KDL Tree
    this->joint_names = joint_names;
    if(this->joint_names.empty())
        this->joint_names = jointNamesFromTree(full_tree);

    // If no base frame is given, take it from KDL tree
    this->base_frame = base_frame;
    if(this->base_frame.empty())
        this->base_frame = full_tree.getRootSegment()->second.segment.getName();

    return true;
}

bool KinematicRobotModelKDL::addTree(const KDL::Tree& tree, const std::string& hook, const base::Pose &pose){

    if(full_tree.getNrOfSegments() == 0)
        full_tree = tree;
    else{
        if(!hasFrame(hook)){
            LOG_ERROR("Hook name is %s, but this segment does not exist in tree", hook.c_str());
            return false;
        }

        std::string root = tree.getRootSegment()->first;
        addVirtual6DoFJoint(hook, root, pose);

        if(!full_tree.addTree(tree, root)){
            LOG_ERROR("Unable to attach tree with root segment %s", root.c_str());
            return false;
        }
    }

    return true;
}

void KinematicRobotModelKDL::createChain(const std::string &root_frame, const std::string &tip_frame){
    KDL::Chain chain;
    if(!full_tree.getChain(root_frame, tip_frame, chain)){
        LOG_ERROR("Unable to extract kinematics chain from %s to %s from KDL tree", root_frame.c_str(), tip_frame.c_str());
        throw std::invalid_argument("Invalid robot model config");
    }

    const std::string chain_id = chainID(root_frame, tip_frame);

    KinematicChainKDLPtr kin_chain = std::make_shared<KinematicChainKDL>(chain, root_frame, tip_frame);
    kin_chain->update(current_joint_state);
    kdl_chain_map[chain_id] = kin_chain;

    jac_map[chain_id]     = base::MatrixXd(6, kin_chain->joint_names.size());
    jac_dot_map[chain_id] = base::MatrixXd(6, kin_chain->joint_names.size());
}

bool KinematicRobotModelKDL::addVirtual6DoFJoint(const std::string &hook, const std::string& tip, const base::Pose& initial_pose){


    const KDL::Joint::JointType virtual_joint_types[6] = {KDL::Joint::TransX, KDL::Joint::TransY, KDL::Joint::TransZ,
                                                          KDL::Joint::RotZ, KDL::Joint::RotY, KDL::Joint::RotX};

    KDL::Chain chain;
    for(int i = 0; i < 6; i++){
        std::string joint_name = tip + virtual_joint_names[i];
        chain.addSegment(KDL::Segment(joint_name, KDL::Joint(joint_name, virtual_joint_types[i]), KDL::Frame::Identity()));
        virtual_joint_state.names.push_back(joint_name);
    }
    virtual_joint_state.elements.resize(virtual_joint_state.names.size());
    chain.addSegment(KDL::Segment(tip, KDL::Joint(KDL::Joint::None))); // Don't forget to add the actual tip segment to the chain

    base::samples::RigidBodyStateSE3 cs;
    cs.frame_id = hook;
    cs.pose = initial_pose;
    cs.twist.setZero();
    cs.time = base::Time::now();
    updateVirtual6DoFJoint(cs, tip);
    robot_models_state.names.push_back(tip);
    robot_models_state.elements.push_back(cs);

    if(!full_tree.addChain(chain, hook)){
        LOG_ERROR("Unable to attach chain to tree segment %s", hook.c_str());
        return false;
    }
    return true;
}

void KinematicRobotModelKDL::updateVirtual6DoFJoint(const base::RigidBodyStateSE3& state, const std::string &tip_frame){

    base::JointState js;
    base::Vector3d euler = base::getEuler(state.pose.orientation);
    for(int i = 0; i < 3; i++){
        std::string name = tip_frame + virtual_joint_names[i];
        js.position = state.pose.position(i);
        js.speed = state.twist.linear(i);
        virtual_joint_state[virtual_joint_state.mapNameToIndex(name)] = js;

        name = tip_frame + virtual_joint_names[i+3];
        js.position = euler(i);
        js.speed = state.twist.angular(i);
        virtual_joint_state[virtual_joint_state.mapNameToIndex(name)] = js;
    }    
}

void KinematicRobotModelKDL::update(const base::samples::Joints& joint_state,
                                    const base::samples::RigidBodyStatesSE3& virtual_joint_states){

    current_joint_state = joint_state;

    // Update virtual joints
    for(size_t i = 0; i < virtual_joint_states.size(); i++){

        std::string name = virtual_joint_states.names[i];
        base::RigidBodyStateSE3 elem = virtual_joint_states.elements[i];
        base::Time time = virtual_joint_states.time;

        if(!hasFrame(name)){
            LOG_ERROR("Trying to update virtual tree element '%s', which is not part of the robot model", name.c_str());
            throw std::runtime_error("Invalid Cartesian state");
        }

        robot_models_state[name] = elem;
        updateVirtual6DoFJoint(elem, name);
        if(time > current_joint_state.time)
            current_joint_state.time = time;
    }

    // Push current virtual joint state into overall joint states
    for(const auto& n : virtual_joint_state.names)
        current_joint_state.names.push_back(n);
    for(const auto& e : virtual_joint_state.elements)
        current_joint_state.elements.push_back(e);

    // update all kinematic chains
    for(const auto& it : kdl_chain_map)
        it.second->update(current_joint_state);
}

const base::samples::RigidBodyStateSE3 &KinematicRobotModelKDL::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("KinematicRobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    if(kdl_chain_map.count(chainID(root_frame, tip_frame)) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chainID(root_frame, tip_frame)];
    return kdl_chain->rigidBodyState();
}

const base::samples::Joints& KinematicRobotModelKDL::jointState(const std::vector<std::string> &joint_names){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("KinematicRobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jointState()");
    }

    joint_state_out.resize(joint_names.size());
    joint_state_out.names = joint_names;
    joint_state_out.time = current_joint_state.time;

    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            joint_state_out[i] = current_joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("KinematicRobotModelKDL: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state_out;
}

const base::MatrixXd& KinematicRobotModelKDL::jacobian(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("KinematicRobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobian()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];

    jac_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_map[chain_id].col(idx) = kdl_chain->jacobian.data.col(j);
    }
    return jac_map[chain_id];
}


const base::MatrixXd &KinematicRobotModelKDL::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("KinematicRobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];

    jac_dot_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_dot_map[chain_id].col(idx) = kdl_chain->jacobian_dot.data.col(j);
    }
    return jac_dot_map[chain_id];
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().count(name) > 0;
}

std::vector<std::string> KinematicRobotModelKDL::jointNamesFromTree(const KDL::Tree &tree) const{

    std::vector<std::string> joint_names;
    KDL::SegmentMap::const_iterator it;
    const KDL::SegmentMap& segments = tree.getSegments();
    for(const auto &it : tree.getSegments()){
        KDL::Joint joint = it.second.segment.getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
    return joint_names;
}

}
