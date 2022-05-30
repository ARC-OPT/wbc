#include "RobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../../core/RobotModelConfig.hpp"
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <algorithm>
#include <tools/URDFTools.hpp>
#include <kdl/chaindynparam.hpp>
#include <fstream>

namespace wbc{

RobotModelRegistry<RobotModelKDL> RobotModelKDL::reg("kdl");

RobotModelKDL::RobotModelKDL(){
}

RobotModelKDL::~RobotModelKDL(){
}

void RobotModelKDL::clear(){
    full_tree = KDL::Tree();
    kdl_chain_map.clear();
    actuated_joint_names.clear();
    current_joint_state.clear();
    contact_points.clear();
    base_frame="";
    gravity = base::Vector3d(0,0,-9.81);
    has_floating_base = false;
    joint_limits.clear();
    robot_urdf.reset();
    joint_names_floating_base.clear();
}

bool RobotModelKDL::configure(const RobotModelConfig& cfg){

    clear();

    // 1. Load Robot Model

    robot_model_config = cfg;

    if(!cfg.submechanism_file.empty()){
        LOG_ERROR("You passed a submechanism file, but RobotModelKDL does not support submechanisms. Use RobotModelHyrodyn instead!");
        return false;
    }

    std::ifstream stream(cfg.file.c_str());
    if (!stream){
        LOG_ERROR("File %s does not exist", cfg.file.c_str());
        return false;
    }

    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }

    // Blacklist not required joints
    if(!URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist))
        return false;

    // Joint names from URDF without floating base and without blacklisted joints
    std::vector<std::string> joint_names_urdf = URDFTools::jointNamesFromURDF(robot_urdf);

    // Add floating base
    has_floating_base = cfg.floating_base;
    if(has_floating_base)
        joint_names_floating_base = URDFTools::addFloatingBaseToURDF(robot_urdf, cfg.world_frame_id);

    // Read Joint Limits
    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    // If joint names is empty in config, use all joints from URDF
    independent_joint_names = cfg.joint_names;
    if(independent_joint_names.empty())
        independent_joint_names = joint_names_floating_base + joint_names_urdf;

    // If actuated joint names is empty in config, assume that all joints are actuated
    actuated_joint_names = cfg.actuated_joint_names;
    if(actuated_joint_names.empty()){
        if(cfg.joint_names.empty())
            actuated_joint_names = joint_names_urdf;
        else
            actuated_joint_names = cfg.joint_names;
    }

    current_joint_state.elements.resize(independent_joint_names.size());
    current_joint_state.names = independent_joint_names;

    // Parse KDL Tree
    if(!kdl_parser::treeFromUrdfModel(*robot_urdf, full_tree)){
        LOG_ERROR("Unable to load KDL Tree from file %s", cfg.file.c_str());
        return false;
    }

    // 2. Verify consistency of URDF and config

    // Check correct floating base names first, if a floating base is available
    if(has_floating_base){
        for(size_t i = 0; i < 6; i++){
            if(jointNames()[i] != joint_names_floating_base[i]){
                LOG_ERROR_S << "If you set 'floating_base' to 'true', the first six entries in joint_names have to be: \n"
                            << "   floating_base_trans_x, floating_base_trans_y, floating_base_trans_z\n"
                            << "   floating_base_rot_x,   floating_base_rot_y,   floating_base_rot_z\n"
                            << "Alternatively you can leave joint_names empty, in which case the joint names will be taken from URDF";
                return false;
            }
        }
    }
    // All non-fixed URDF joint names have to be configured in cfg.joint_names and vice versa
    joint_names_urdf = URDFTools::jointNamesFromURDF(robot_urdf);
    for(const std::string& n : jointNames()){
        if(std::find(joint_names_urdf.begin(), joint_names_urdf.end(), n) == joint_names_urdf.end()){
            LOG_ERROR_S << "Joint " << n << " has been configured in joint_names, but is not a non-fixed joint in the robot URDF"<<std::endl;
            return false;
        }
    }
    for(const std::string& n : joint_names_urdf){
        if(!hasJoint(n)){
            LOG_ERROR_S << "Joint " << n << " is a non-fixed joint in the URDF model, but has not been configured in joint names"<<std::endl;
            return false;
        }
    }
    // All actuated joint names have to exists in robot model
    for(const std::string& n : actuated_joint_names){
        if(!hasJoint(n)){
            LOG_ERROR_S << "Joint " << n << " has been configured in actuated_joint_names, but is not a non-fixed joint in the robot URDF"<<std::endl;
            return false;
        }
    }
    // All contact point have to be a valid link in the robot URDF
    for(auto c : cfg.contact_points.names){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }

    // 3. Set initial floating base state
    if(has_floating_base){
        if(cfg.floating_base_state.hasValidPose()){
            base::samples::RigidBodyStateSE3 rbs;
            rbs.pose = cfg.floating_base_state.pose;
            if(cfg.floating_base_state.hasValidTwist())
                rbs.twist = cfg.floating_base_state.twist;
            else
                rbs.twist.setZero();
            if(cfg.floating_base_state.hasValidAcceleration())
                rbs.acceleration = cfg.floating_base_state.acceleration;
            else
                rbs.acceleration.setZero();
            rbs.time = base::Time::now();
            rbs.frame_id = cfg.world_frame_id;
            try{
                updateFloatingBase(rbs, joint_names_floating_base, current_joint_state);
            }
            catch(std::runtime_error e){
                return false;
            }
        }
        else{
            LOG_ERROR("If you set floating_base to true, you have to provide a valid floating_base_state (at least a position/orientation)");
            return false;
        }
    }

    // 4. Create data structures

    q.resize(noOfJoints());
    qdot.resize(noOfJoints());
    qdotdot.resize(noOfJoints());
    tau.resize(noOfJoints());
    zero.resize(noOfJoints());
    zero.data.setZero();
    base_frame =  robot_urdf->getRoot()->name;
    contact_points = cfg.contact_points.names;
    active_contacts = cfg.contact_points;
    joint_space_inertia_mat.resize(noOfJoints(), noOfJoints());
    bias_forces.resize(noOfJoints());
    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, jointIndex(actuated_joint_names[i])) = 1.0;

    for(const auto &it : full_tree.getSegments()){
        KDL::Joint jnt = it.second.segment.getJoint();
        if(jnt.getType() != KDL::Joint::None)
            joint_idx_map_kdl[jnt.getName()] = GetTreeElementQNr(it.second);
    }

    // 5. Print some debug info

    LOG_DEBUG("------------------- WBC RobotModelKDL -----------------");
    LOG_DEBUG_S << "Robot Name " << robot_urdf->getName() << std::endl;
    LOG_DEBUG_S << "Floating base robot: " << has_floating_base << std::endl;
    if(has_floating_base){
        LOG_DEBUG_S << "Floating base pose: " << std::endl;
        LOG_DEBUG_S << "Pos: " << cfg.floating_base_state.pose.position.transpose() << std::endl;
        LOG_DEBUG_S << "Ori: " << cfg.floating_base_state.pose.orientation.coeffs().transpose() << std::endl;
        LOG_DEBUG_S << "World frame: " << cfg.world_frame_id << std::endl;
    }
    LOG_DEBUG("Joint Names");
    for(auto n : jointNames())
        LOG_DEBUG_S << n << std::endl;
    LOG_DEBUG("Actuated Joint Names");
    for(auto n : actuatedJointNames())
        LOG_DEBUG_S << n << std::endl;
    LOG_DEBUG("Joint Limits");
    for(auto n : joint_limits.names)
        LOG_DEBUG_S << n << ": Max. Pos: " << joint_limits[n].max.position << ", "
                         << "  Min. Pos: " << joint_limits[n].min.position << ", "
                         << "  Max. Vel: " << joint_limits[n].max.speed    << ", "
                         << "  Max. Eff: " << joint_limits[n].max.effort   << std::endl;
    LOG_DEBUG("------------------------------------------------------------");

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

    if(joint_state.elements.size() != joint_state.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    for(size_t i = 0; i < noOfActuatedJoints(); i++){
        const std::string& name = actuated_joint_names[i];
        std::size_t idx;
        try{
            idx = joint_state.mapNameToIndex(name);
        }
        catch(base::samples::Joints::InvalidName e){
            LOG_ERROR_S<<"Robot model contains joint "<<name<<" but this joint is not in joint state vector"<<std::endl;
            throw e;
        }
        current_joint_state[name] = joint_state[idx];
    }
    current_joint_state.time = joint_state.time;
    // Convert floating base to joint state
    if(has_floating_base)
        updateFloatingBase(_floating_base_state, joint_names_floating_base, current_joint_state);

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

const base::samples::Joints& RobotModelKDL::jointState(const std::vector<std::string> &joint_names){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
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
            LOG_ERROR("RobotModelKDL: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state_out;
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
    kdl_chain->calculateForwardKinematics();

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
    kdl_chain->calculateSpaceJacobian();

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
    kdl_chain->calculateBodyJacobian();

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
    kdl_chain->calculateJacobianDot();

    jac_dot_map[chain_id].resize(6,noOfJoints());
    jac_dot_map[chain_id].setZero(6,noOfJoints());
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_dot_map[chain_id].col(idx) = kdl_chain->jacobian_dot.data.col(j);
    }
    return jac_dot_map[chain_id];
}

const base::Acceleration &RobotModelKDL::spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame){
    tmp_acc = jacobianDot(root_frame, tip_frame)*qdot.data;
    spatial_acc_bias = base::Acceleration(tmp_acc.segment(0,3), tmp_acc.segment(3,3));
    return spatial_acc_bias;
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


void RobotModelKDL::recursiveCOM( const KDL::SegmentMap::const_iterator& currentSegment,
                                  const base::samples::Joints& status, const KDL::Frame& frame,
                                  double& mass, KDL::Vector& cog)
{
    double jointPosition = 0.0; // Joint position of the current joint
#ifdef KDL_USE_NEW_TREE_INTERFACE
    KDL::Segment segment = currentSegment->second->segment;
#else
    KDL::Segment segment = currentSegment->second.segment;
#endif
    // If the segment has a real joint, get the joint position
    if( segment.getJoint().getType() != KDL::Joint::None ) {
        try {
            jointPosition = status[segment.getJoint().getName() ].position;
        } catch (const std::runtime_error& error)
        {
            LOG_WARN("There is not information for this joint: ", segment.getJoint().getName().c_str());
        }
    }

    // Transform to the frame of the current segment
    KDL::Frame currentFrame = frame * segment.pose( jointPosition );
    // Get the COG of the current segemnt in the KDL tree
    KDL::Vector currentCOG = segment.getInertia().getCOG();
    // Gets the current mass
    double currentMass = segment.getInertia().getMass();

    // Computes the cog
    cog += currentMass * ( currentFrame * currentCOG );
    // Appends to the total mass
    mass += currentMass;

    // Iterates through all the child segments
    std::vector< KDL::SegmentMap::const_iterator >::const_iterator childSegment;
#ifdef KDL_USE_NEW_TREE_INTERFACE
    for( childSegment = currentSegment->second->children.begin();
            childSegment != currentSegment->second->children.end();
#else
    for( childSegment = currentSegment->second.children.begin();
            childSegment != currentSegment->second.children.end();
#endif
            ++childSegment)
    {
        // Calls this functiion again with the child segment
        recursiveCOM( *childSegment, status, currentFrame, mass, cog );
    }
}

const base::samples::RigidBodyStateSE3& RobotModelKDL::centerOfMass(){
    double mass = 0.0; // to get the total mass
    KDL::Frame frame = KDL::Frame::Identity(); // Transformation of the last frame

    // Computes the COG of the complete robot
    KDL::Vector cog_pos;
    recursiveCOM( full_tree.getRootSegment(), current_joint_state, frame, mass, cog_pos );

    // Returns the COG
    com_rbs.frame_id = base_frame;
    com_rbs.pose.position = base::Vector3d( cog_pos.x(), cog_pos.y(), cog_pos.z() ) / mass;
    com_rbs.pose.orientation.setIdentity();
    com_rbs.time = current_joint_state.time;
    return com_rbs;
}

uint RobotModelKDL::jointIndex(const std::string &joint_name){
    uint idx = std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) - current_joint_state.names.begin();
    if(idx >= current_joint_state.names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}

bool RobotModelKDL::hasLink(const std::string &link_name){
    for(auto l  : robot_urdf->links_)
        if(l.second->name == link_name)
            return true;
    return false;
}

bool RobotModelKDL::hasJoint(const std::string &joint_name){
    return std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) != current_joint_state.names.end();
}

bool RobotModelKDL::hasActuatedJoint(const std::string &joint_name){
    return std::find(actuated_joint_names.begin(), actuated_joint_names.end(), joint_name) != actuated_joint_names.end();
}

void RobotModelKDL::computeInverseDynamics(base::commands::Joints &solver_output){
    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    KDL::TreeIdSolver_RNE solver(full_tree, KDL::Vector(gravity[0], gravity[1], gravity[2]));
    KDL::WrenchMap w_map;
    for(auto n : contact_wrenches.names)
        w_map[n] = KDL::Wrench(KDL::Vector(contact_wrenches[n].force[0],
                                           contact_wrenches[n].force[1],
                                           contact_wrenches[n].force[2]),
                               KDL::Vector(contact_wrenches[n].torque[0],
                                           contact_wrenches[n].torque[1],
                                           contact_wrenches[n].torque[2]));
    int ret = solver.CartToJnt(q, qdot, qdotdot, w_map, tau);
    if(ret != 0)
        throw(std::runtime_error("Unable to compute Tree Inverse Dynamics. Error Code is " + std::to_string(ret)));

    for(uint i = 0; i < noOfActuatedJoints(); i++){
        const std::string& name = actuated_joint_names[i];
        solver_output[name].effort = tau(joint_idx_map_kdl[name]);
    }
}
}
