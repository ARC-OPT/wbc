#include "KinematicRobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../core/RobotModelConfig.hpp"
#include <urdf_parser/urdf_parser.h>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <algorithm>

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
    joint_limits.clear();
}


bool KinematicRobotModelKDL::configure(const std::string& model_filename,
                                       const std::vector<std::string> &joint_names){
    clear();

    RobotModelConfig cfg;
    cfg.file = model_filename;
    cfg.type = ModelType::ROBOT;
    cfg.joint_names = joint_names;
    cfg.actuated_joint_names = joint_names;
    std::vector<RobotModelConfig> configs;
    configs.push_back(cfg);
    return configure(configs);
}

bool KinematicRobotModelKDL::configure(const std::vector<RobotModelConfig>& model_config){

    clear();

    if(model_config.empty()){
        LOG_ERROR("The passed robot model config is empty! You have to pass at least one robot model!");
        return false;
    }

    for(int i = 0; i < model_config.size(); i++){

        const RobotModelConfig& cfg = model_config[i];

        if(cfg.type == ModelType::UNSET){
            LOG_ERROR("Robot Model Config #%i is invalid: Model type unset", i);
            return false;
        }
        if(cfg.file.empty()){
            LOG_ERROR("Robot Model Config #%i is invalid: No model file given in robot model config", i);
            return false;
        }
        if(cfg.joint_names.empty()){
            LOG_ERROR("Robot Model Config #%i is invalid: Joint names vector is empty!", i);
            return false;
        }

        if(cfg.hook.empty() && i > 0){
            LOG_ERROR("Robot Model Config #%i is invalid: Hook is empty! To which link do you want to attach this model?", i);
            return false;
        }

        if(cfg.type == ModelType::VIRTUAL_6_DOF_JOINT){
            if(!cfg.actuated_joint_names.empty()){
                LOG_ERROR("Robot Model Config #%i is invalid: If model type is VIRTUAL_6_DOF_JOINT, the model must NOT contain actuated joints", i);
                return false;
            }

            std::vector<std::string> joint_names_urdf = jointNamesFromURDF(cfg.file);
            if(!std::is_permutation(cfg.joint_names.begin(), cfg.joint_names.end(), joint_names_urdf.begin())){
                LOG_ERROR("Robot Model Config #%i is invalid: Given joint names must match joint names in URDF file", i);
                return false;
            }
        }

        jointLimitsFromURDF(cfg.file, joint_limits);

        KDL::Tree tree;
        if(!kdl_parser::treeFromFile(cfg.file, tree)){
            LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
            return false;
        }

        if(full_tree.getNrOfSegments() == 0 && full_tree.getRootSegment()->first == "root"){
            full_tree = tree;
            LOG_INFO_S<<"Added full tree with root "<<tree.getRootSegment()->first<<" and no. of segments: "<<tree.getNrOfSegments()<<std::endl;
        }
        else{
            if(!hasFrame(cfg.hook)){
                LOG_ERROR("Hook name is %s, but this segment does not exist in tree", cfg.hook.c_str());
                return false;
            }
            KDL::Segment root = tree.getRootSegment()->second.segment;
            std::string root_name = root.getName();

            // KDL::Tree::addTree() omits the root segment of the attached tree for some reason, so we have to explicity add it here
            full_tree.addSegment(root, cfg.hook);

            if(!full_tree.addTree(tree, root_name)){
                LOG_ERROR("Unable to attach tree with root segment %s", root_name.c_str());
                return false;
            }

            LOG_INFO_S<<"Added new tree with root "<<root_name<<" to hook "<<cfg.hook<<std::endl;
        }

        for(size_t i = 0; i < cfg.joint_names.size(); i++){
            current_joint_state.elements.push_back(base::JointState());
            current_joint_state.names.push_back(cfg.joint_names[i]);
        }
        for(size_t i = 0; i < cfg.actuated_joint_names.size(); i++)
            actuated_joint_names.push_back(cfg.actuated_joint_names[i]);

        if(cfg.type == ModelType::VIRTUAL_6_DOF_JOINT)
            toJointState(cfg.initial_state, robotNameFromURDF(cfg.file), current_joint_state);

    }

    base_frame = full_tree.getRootSegment()->second.segment.getName();

    q.resize(noOfJoints());
    qdot.resize(noOfJoints());
    qdotdot.resize(noOfJoints());
    tau.resize(noOfJoints());
    joint_space_inertia_mat.resize(noOfJoints(), noOfJoints());
    bias_forces.resize(noOfJoints());
    zero.resize(noOfJoints());
    zero.data.setZero();

    for(const auto &it : full_tree.getSegments()){
        KDL::Joint jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None)
            joint_idx_map_kdl[jnt.getName()] = GetTreeElementQNr(it.second);
    }

    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, jointIndex(actuated_joint_names[i])) = 1.0;

    LOG_INFO_S<<"Actuated Joint Names: "<<std::endl;
    for(auto n : actuatedJointNames())
        LOG_INFO_S << n;
    LOG_INFO_S<<std::endl;

    LOG_INFO_S<<"All Joint Names: "<<std::endl;
    for(auto n : jointNames())
        LOG_INFO_S << n;
    LOG_INFO_S<<std::endl;

    LOG_INFO("Full KDL Tree");
    if(getenv("BASE_LOG_LEVEL")){
        if(std::string(getenv("BASE_LOG_LEVEL")) == "INFO" || std::string(getenv("BASE_LOG_LEVEL")) == "DEBUG"){
            KDL::TreeElement root = full_tree.getRootSegment()->second;
            std::cout<<"Root Link: "<<root.segment.getName()<<std::endl;
            printTree(root);
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

    LOG_INFO_S<<"Added chain "<<root_frame<<" --> "<<tip_frame<<std::endl;
}

void KinematicRobotModelKDL::toJointState(const base::samples::RigidBodyStateSE3& rbs, const std::string &name, base::samples::Joints& joint_state){

    if(!rbs.hasValidPose() ||
       !rbs.hasValidTwist() ||
       !rbs.hasValidAcceleration()){
       LOG_ERROR("Unable to convert rigid body state of virtual 6 DoF joint to joint state! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion");
       throw std::runtime_error("Invalid Rigid Body State");
    }

    base::JointState js;
    base::Pose tmp_a, tmp_b;
    tmp_a.position.setZero();
    tmp_a.orientation = rbs.pose.orientation;
    tmp_b.position.setZero();
    tmp_b.orientation.setIdentity();
    base::Vector3d euler = rbs.pose.toTransform().rotation().eulerAngles(0,1,2); // TODO: Use Rotation Vector instead?
    for(int j = 0; j < 3; j++){
        js.position = rbs.pose.position(j);
        js.speed = rbs.twist.linear(j);
        js.acceleration = rbs.acceleration.linear(j);
        joint_state[name + virtual_joint_names[j]] = js;

        js.position = euler(j);
        js.speed = rbs.twist.angular(j);
        js.acceleration = rbs.acceleration.angular(j);
        joint_state[name + virtual_joint_names[j+3]] = js;
    }
}

void KinematicRobotModelKDL::update(const base::samples::Joints& joint_state,
                                    const base::NamedVector<base::samples::RigidBodyStateSE3>& virtual_joint_states){


    // Update actuated joints
    for(size_t i = 0; i < joint_state.size(); i++){
        uint idx = current_joint_state.mapNameToIndex(joint_state.names[i]);
        current_joint_state[idx] = joint_state[i];
        current_joint_state[idx].acceleration = 0.0;
    }
    current_joint_state.time = joint_state.time;

    // Update virtual joints
    for(size_t i = 0; i < virtual_joint_states.size(); i++){
        toJointState(virtual_joint_states[i], virtual_joint_states.names[i], current_joint_state);

        if(virtual_joint_states[i].time > current_joint_state.time) // Use the latest time stamp of incoming data
            current_joint_state.time = virtual_joint_states[i].time;
    }

    // update all kinematic chains
    for(const auto& it : kdl_chain_map)
        it.second->update(current_joint_state);

    // Update KDL data types
    for(const auto &it : full_tree.getSegments()){
        const KDL::Joint& jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None){
            uint idx = GetTreeElementQNr(it.second);
            const std::string& name = jnt.getName();

            if(!hasJoint(name)){
                LOG_ERROR_S << "Joint " << name << " is a non-fixed joint in the KDL Tree, but it is not in the joint state vector."
                            << "You should either set the joint to 'fixed' in your URDF file or provide a valid joint state for it" << std::endl;
                throw std::runtime_error("Incomplete Joint State");
            }

            q(idx)       = current_joint_state[name].position;
            qdot(idx)    = current_joint_state[name].speed;
            qdotdot(idx) = current_joint_state[name].acceleration;
        }
    }

    computeJointSpaceInertiaMatrix();
    computeBiasForces();
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

const base::MatrixXd& KinematicRobotModelKDL::fullJacobian(const std::string &root_frame, const std::string &tip_frame){

    const std::string chain_id = chainID(root_frame, tip_frame);
    const base::MatrixXd& jac = jacobian(root_frame,tip_frame);
    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];

    full_jac_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        full_jac_map[chain_id].col(idx) = jac.col(j);
    }
    return full_jac_map[chain_id];
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
    jac_map[chain_id] = kdl_chain->jacobian.data;

    return jac_map[chain_id];
}

const base::MatrixXd &KinematicRobotModelKDL::fullJacobianDot(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("KinematicRobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];

    full_jac_dot_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        full_jac_dot_map[chain_id].col(idx) = kdl_chain->jacobian_dot.data.col(j);
    }
    return full_jac_dot_map[chain_id];
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
    jac_dot_map[chain_id] = kdl_chain->jacobian_dot.data;

    return jac_dot_map[chain_id];
}

void KinematicRobotModelKDL::computeBiasForces(){
    // Use ID solver with zero joint accelerations and zero external wrenches to get bias forces/torques
    KDL::TreeIdSolver_RNE solver(full_tree, KDL::Vector(gravity(0), gravity(1), gravity(2)));
    solver.CartToJnt(q, qdot, zero, std::map<std::string,KDL::Wrench>(), tau);

    for(const auto &it : full_tree.getSegments()){
        const KDL::Joint& jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None){
            const std::string& name = jnt.getName();
            uint idx = GetTreeElementQNr(it.second);
            bias_forces[current_joint_state.mapNameToIndex(name)] = tau(idx);
        }
    }
}

void KinematicRobotModelKDL::computeJointSpaceInertiaMatrix(){
    joint_space_inertia_mat.setZero();

    // Use ID solver with zero bias and external wrenches to compute joint space inertia matrix column by column.
    // TODO: Switch to more efficient method!
    KDL::TreeIdSolver_RNE solver(full_tree, KDL::Vector::Zero());
    for(const auto &it : full_tree.getSegments()){
        const KDL::Joint& jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None){
            const std::string& name = jnt.getName();
            qdotdot.data.setZero();
            qdotdot(GetTreeElementQNr(it.second)) = 1;
            int ret = solver.CartToJnt(q, zero, qdotdot, std::map<std::string,KDL::Wrench>(), tau);
            if(ret != 0)
                throw(std::runtime_error("Unable to compute Tree Inverse Dynamics in joint space inertia matrix computation. Error Code is " + std::to_string(ret)));
            uint idx_col = current_joint_state.mapNameToIndex(name);
            for(int j = 0; j < noOfJoints(); j++){
                uint idx_row = current_joint_state.mapNameToIndex(current_joint_state.names[j]);
                joint_space_inertia_mat(idx_row, idx_col) = tau(idx_row);
            }
        }
    }
}

bool KinematicRobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().count(name) > 0;
}

bool KinematicRobotModelKDL::hasJoint(const std::string &name){
    return std::find(current_joint_state.names.begin(), current_joint_state.names.end(), name) != current_joint_state.names.end();
}


const std::string &KinematicRobotModelKDL::robotNameFromURDF(const std::string &filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    return urdf_model->getName();
}

const std::string &KinematicRobotModelKDL::rootLinkFromURDF(const std::string &filename){
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(filename, tree))
        throw std::runtime_error("Unable to load URDF from file " + filename);
    return tree.getRootSegment()->second.segment.getName();
}

std::vector<std::string> KinematicRobotModelKDL::jointNamesFromURDF(const std::string &filename){
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(filename, tree))
        throw std::runtime_error("Unable to load URDF from file " + filename);
    return KinematicRobotModelKDL::jointNamesFromTree(tree);
}

std::vector<std::string> KinematicRobotModelKDL::jointNamesFromTree(const KDL::Tree &tree){
    std::vector<std::string> j_names;
    KDL::SegmentMap::const_iterator it;
    const KDL::SegmentMap& segments = tree.getSegments();
    for(const auto &it : tree.getSegments()){
        KDL::Joint joint = it.second.segment.getJoint();
        if(joint.getType() != KDL::Joint::None)
            j_names.push_back(joint.getName());
    }
    return j_names;
}

void KinematicRobotModelKDL::jointLimitsFromURDF(const std::string& filename, base::JointLimits& limits){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw std::invalid_argument("Cannot load URDF from file " + filename);

    std::map<std::string, urdf::JointSharedPtr>::const_iterator it;
    for(it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        const urdf::JointSharedPtr &joint = it->second;
        base::JointLimitRange range;

        if(joint->limits){
            range.max.position = joint->limits->upper;
            range.min.position = joint->limits->lower;
            range.max.speed = joint->limits->velocity;
            range.min.speed = -joint->limits->velocity;
            range.max.effort = joint->limits->effort;
            range.min.effort = -joint->limits->effort;

            try{
                limits.getElementByName(it->first);
            }
            catch(base::JointLimits::InvalidName e){
                limits.names.push_back(it->first);
                limits.elements.push_back(range);

                LOG_INFO_S<<"Added joint limit for joint "<<it->first<<" Max. Pos: "<<range.max.position
                         <<" Min. Pos: "<<range.min.position<<" Max. Vel: "<<range.max.speed<<" Max. Effort: "<<range.max.effort<<std::endl;
            }
        }
    }
}

void KinematicRobotModelKDL::printTree(const KDL::TreeElement& tree_element, int level){
    level+=1;
    for(auto c : tree_element.children){
        for(int j=0;j<level;j++) std::cout << " "; //indent
        KDL::Segment seg = c->second.segment;
        std::cout<<"Link: "<<seg.getName()<<" Parent: "<<c->second.parent->second.segment.getName()<<std::endl;
        printTree(c->second, level);
    }
}

}
