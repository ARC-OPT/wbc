#include "RobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../core/RobotModelConfig.hpp"
#include <urdf_parser/urdf_parser.h>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <algorithm>
#include "URDFTools.hpp"

namespace wbc{

RobotModelKDL::RobotModelKDL(){
}

RobotModelKDL::~RobotModelKDL(){
}

void RobotModelKDL::clear(){
    RobotModel::clear();
    full_tree = KDL::Tree();
    kdl_chain_map.clear();
}

bool RobotModelKDL::configure(const std::vector<RobotModelConfig>& model_config){

    clear();

    if(model_config.size() != 1 || model_config[0].type != URDF){
        LOG_ERROR("Robot model config has to contain exactly one URDF model");
        return false;
    }

    const RobotModelConfig &cfg = model_config[0];

    urdf::ModelInterfaceSharedPtr robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }

    has_floating_base = cfg.floating_base;
    if(has_floating_base)
        floating_base_names = URDFTools::addFloatingBaseToURDF(robot_urdf, cfg.world_frame_id);

    if(!kdl_parser::treeFromUrdfModel(*robot_urdf, full_tree)){
        LOG_ERROR("Unable to parse URDF model to KDL");
        return false;
    }

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    for(size_t i = 0; i < cfg.joint_names.size(); i++){
        current_joint_state.elements.push_back(base::JointState());
        current_joint_state.names.push_back(cfg.joint_names[i]);
    }
    for(size_t i = 0; i < cfg.actuated_joint_names.size(); i++)
        actuated_joint_names.push_back(cfg.actuated_joint_names[i]);

    if(has_floating_base)
        updateFloatingBase(cfg.floating_base_state, current_joint_state);

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

    if(getenv("BASE_LOG_LEVEL")){
        if(std::string(getenv("BASE_LOG_LEVEL")) == "INFO" || std::string(getenv("BASE_LOG_LEVEL")) == "DEBUG"){
            std::cout<<"Actuated Joints: "<<std::endl;
            for(auto n : actuatedJointNames())
                std::cout<<n;
            std::cout<<std::endl;

            std::cout<<"All Joints: "<<std::endl;
            for(auto n : jointNames())
                std::cout << n;
            std::cout<<std::endl;

            std::cout<<"URDF Tree"<<std::endl;
            KDL::TreeElement root = full_tree.getRootSegment()->second;
            std::cout<<"Root Link: "<<root.segment.getName()<<std::endl;
            URDFTools::printTree(robot_urdf->getRoot());
        }
    }

    return true;
}

void RobotModelKDL::createChain(const std::string &root_frame, const std::string &tip_frame){
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

void RobotModelKDL::updateFloatingBase(const base::RigidBodyStateSE3& rbs, base::samples::Joints& joint_state){

    if(!rbs.hasValidPose() ||
       !rbs.hasValidTwist() ||
       !rbs.hasValidAcceleration()){
       LOG_ERROR("Invalid status of floating base given! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion)");
       throw std::runtime_error("Invalid floating base status");
    }

    base::JointState js;
    base::Vector3d euler = rbs.pose.toTransform().rotation().eulerAngles(0,1,2); // TODO: Use Rotation Vector instead?
    for(int j = 0; j < 3; j++){
        js.position = rbs.pose.position(j);
        js.speed = rbs.twist.linear(j);
        js.acceleration = rbs.acceleration.linear(j);
        joint_state[floating_base_names[j]] = js;

        js.position = euler(j);
        js.speed = rbs.twist.angular(j);
        js.acceleration = rbs.acceleration.angular(j);
        joint_state[floating_base_names[j+3]] = js;
    }
}

void RobotModelKDL::update(const base::samples::Joints& joint_state,
                           const base::samples::RigidBodyStateSE3& _floating_base_state){


    RobotModel::update(joint_state, _floating_base_state);

    // Convert floating base to joint state
    if(has_floating_base)
        updateFloatingBase(floating_base_state, current_joint_state);

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
}

const base::samples::RigidBodyStateSE3 &RobotModelKDL::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chainID(root_frame, tip_frame)];
    if(kdl_chain->stamp != current_joint_state.time) // Have to recompute!
        kdl_chain->update(current_joint_state);

    return kdl_chain->rigidBodyState();
}

const base::MatrixXd& RobotModelKDL::jacobian(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];
    if(kdl_chain->stamp != current_joint_state.time) // Have to recompute!
        kdl_chain->update(current_joint_state);

    jac_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_map[chain_id].col(idx) = kdl_chain->jacobian.data.col(j);
    }
    return jac_map[chain_id];
}

const base::MatrixXd &RobotModelKDL::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];
    if(kdl_chain->stamp != current_joint_state.time) // Have to recompute!
        kdl_chain->update(current_joint_state);

    jac_dot_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_dot_map[chain_id].col(idx) = kdl_chain->jacobian_dot.data.col(j);
    }
    return jac_dot_map[chain_id];
}

const base::VectorXd &RobotModelKDL::biasForces(){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

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
    return bias_forces;
}

const base::MatrixXd& RobotModelKDL::jointSpaceInertiaMatrix(){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

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
    return joint_space_inertia_mat;
}

bool RobotModelKDL::hasFrame(const std::string &name){
    return full_tree.getSegments().count(name) > 0;
}

bool RobotModelKDL::hasJoint(const std::string &name){
    return std::find(current_joint_state.names.begin(), current_joint_state.names.end(), name) != current_joint_state.names.end();
}


}
