#include "KinematicRobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"

namespace wbc{

KinematicRobotModelKDL::KinematicRobotModelKDL(const std::vector<std::string> &joint_names) :
    joint_names(joint_names){
    if(!joint_names.empty())
        createJointIndexMap(joint_names);
}

KinematicRobotModelKDL::~KinematicRobotModelKDL(){
    current_joint_state.clear();
    current_poses.clear();
    joint_index_map.clear();
    full_tree = KDL::Tree();

    KinematicChainKDLMap::const_iterator it;
    for(it = kdl_chain_map.begin(); it != kdl_chain_map.end(); it++)
        delete it->second;
    kdl_chain_map.clear();
}

void KinematicRobotModelKDL::createJointIndexMap(const std::vector<std::string> &joint_names){
    for(size_t i = 0; i < joint_names.size(); i++)
        joint_index_map[joint_names[i]] = i;
    robot_jacobian.resize(joint_names.size());
}

void KinematicRobotModelKDL::createChain(const std::string &root_frame, const std::string &tip_frame){
    KDL::Chain chain;
    if(!full_tree.getChain(root_frame, tip_frame, chain))
        throw std::invalid_argument("KinematicRobotModelKDL: Unable to extract chain from " + root_frame + " to " + tip_frame + " from KDL Tree");

    KinematicChainKDL* kin_chain = new KinematicChainKDL(chain);
    kin_chain->update(current_joint_state, current_poses);
    kdl_chain_map[root_frame + "_" + tip_frame] = kin_chain;
}

std::vector<std::string> KinematicRobotModelKDL::jointNamesFromTree(KDL::Tree tree){
    std::vector<std::string> joint_names;
    const KDL::SegmentMap &segments = tree.getSegments();

    for(KDL::SegmentMap::const_iterator it = segments.begin(); it != segments.end(); it++){
        const KDL::Joint &joint = it->second.segment.getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
    return joint_names;
}

void KinematicRobotModelKDL::addTree(const KDL::Tree& tree, const std::string& hook, const KDL::Frame &pose){

    if(full_tree.getNrOfSegments() == 0)
        full_tree = tree;
    else{
        if(hook.empty())
            throw std::invalid_argument("KinematicRobotModelKDL::addTree: Unexpected empty hook name. To which segment do you want to attach the tree?");
        if(!hasFrame(hook))
            throw std::invalid_argument("KinematicRobotModelKDL::addTree: Hook name is " + hook + ", but this segment does not exist in tree");

        std::string root = tree.getRootSegment()->first;
        if(!full_tree.addSegment(KDL::Segment(root, KDL::Joint(KDL::Joint::None), pose), hook))
            throw std::invalid_argument("Unable to attach segment " + root + " to existing tree segment " + hook);
        if(!full_tree.addTree(tree, root))
            throw std::invalid_argument("Unable to attach tree with root segment " + root);
    }

    if(joint_names.empty())
        createJointIndexMap(jointNamesFromTree(full_tree));
}

void KinematicRobotModelKDL::update(const base::samples::Joints& joint_state,
                                    const std::vector<base::samples::RigidBodyState>& poses){

    current_joint_state = joint_state;
    current_poses = poses;
    last_update = joint_state.time;

    KinematicChainKDLMap::const_iterator it;
    for(it = kdl_chain_map.begin(); it != kdl_chain_map.end(); it++){
        it->second->update(joint_state, poses);

        // set last update to the latest time available
        for(size_t i = 0; i < poses.size(); i++)
            if(poses[i].time > last_update)
                last_update = poses[i].time;
    }
}

void KinematicRobotModelKDL::rigidBodyState(const std::string &root_frame,
                                            const std::string &tip_frame,
                                            base::samples::RigidBodyState& rigid_body_state){
    if(last_update.isNull())
        throw std::runtime_error("KinematicRobotModelKDL: You have to call update() at least once before requesting kinematic information!");

    // Create chain if it does not exist
    if(kdl_chain_map.count(root_frame + "_" + tip_frame) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDL* kdl_chain = kdl_chain_map[root_frame + "_" + tip_frame];

    rigid_body_state = kdl_chain->rigid_body_state;
    rigid_body_state.sourceFrame = tip_frame;
    rigid_body_state.targetFrame = root_frame;
    rigid_body_state.time = last_update;
}

void KinematicRobotModelKDL::jointState(const std::vector<std::string> &joint_names,
                                        base::samples::Joints& joint_state){
    if(last_update.isNull())
        throw std::runtime_error("KinematicRobotModelKDL: You have to call update() at least once before requesting kinematic information!");

    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    joint_state.time = last_update;

    for(uint i = 0; i < joint_names.size(); i++){
        try{
            joint_state[i] = current_joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            throw std::invalid_argument("KinematicRobotModelKDL: Requested state of joint "
                                        + joint_names[i] + " but this joint does not exist in robot model");
        }
    }
}

void KinematicRobotModelKDL::jacobian(const std::string &root_frame,
                                      const std::string &tip_frame,
                                      base::MatrixXd& jacobian){
    if(last_update.isNull())
        throw std::runtime_error("KinematicRobotModelKDL: You have to call update() at least once before requesting kinematic information!");

    // Create chain if it does not exist
    if(kdl_chain_map.count(root_frame + "_" + tip_frame) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDL* kdl_chain = kdl_chain_map[root_frame + "_" + tip_frame];
    robot_jacobian.data.setZero();

    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){

        const std::string& jt_name =  kdl_chain->joint_names[j];
        if(joint_index_map.count(jt_name) == 0)
            throw std::invalid_argument("Joint with name " + jt_name + " is in kinematic chain between frame " + root_frame +
                                        " and frame " + tip_frame + ", but is not in joint index map");

        robot_jacobian.setColumn(joint_index_map[jt_name], kdl_chain->jacobian.getColumn(j));
    }
    jacobian = robot_jacobian.data;
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().count(name) > 0;
}

}
