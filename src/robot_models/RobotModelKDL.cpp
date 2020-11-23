#include "RobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../core/RobotModelConfig.hpp"
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <algorithm>

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

bool RobotModelKDL::configure(const RobotModelConfig& cfg){

    if(!RobotModel::configure(cfg))
        return false;

    if(!kdl_parser::treeFromUrdfModel(*robot_urdf, full_tree)){
        LOG_ERROR("Unable to load KDL Tree from file %s", cfg.file.c_str());
        return false;
    }

    q.resize(noOfJoints());
    qdot.resize(noOfJoints());
    qdotdot.resize(noOfJoints());
    tau.resize(noOfJoints());
    zero.resize(noOfJoints());
    zero.data.setZero();

    for(const auto &it : full_tree.getSegments()){
        KDL::Joint jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None)
            joint_idx_map_kdl[jnt.getName()] = GetTreeElementQNr(it.second);
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

    LOG_INFO_S<<"Added chain "<<root_frame<<" --> "<<tip_frame<<std::endl;
}

void RobotModelKDL::update(const base::samples::Joints& joint_state,
                           const base::samples::RigidBodyStateSE3& _floating_base_state){


    RobotModel::update(joint_state, _floating_base_state);

    // Convert floating base to joint state
    if(has_floating_base)
        updateFloatingBase(floating_base_state, current_joint_state);

    for(auto c : kdl_chain_map)
        c.second->update(current_joint_state);

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

    return kdl_chain->rigidBodyState();
}

const base::MatrixXd& RobotModelKDL::spaceJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];
    space_jac_map[chain_id].resize(6,noOfJoints());
    space_jac_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        space_jac_map[chain_id].col(idx) = kdl_chain->space_jacobian.data.col(j);
    }
    return space_jac_map[chain_id];
}

const base::MatrixXd& RobotModelKDL::bodyJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];
    body_jac_map[chain_id].resize(6,noOfJoints());
    body_jac_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        body_jac_map[chain_id].col(idx) = kdl_chain->body_jacobian.data.col(j);
    }
    return body_jac_map[chain_id];
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
    jac_dot_map[chain_id].resize(6,noOfJoints());
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

}
