#include "RobotModelKDL.hpp"
#include "KinematicChainKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include "../../core/RobotModelConfig.hpp"
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <algorithm>
#include <tools/URDFTools.hpp>
#include <kdl/chaindynparam.hpp>

namespace wbc{

RobotModelRegistry<RobotModelKDL> RobotModelKDL::reg("kdl");

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

    clear();

    // 1. Load Robot Model

    robot_model_config = cfg;
    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }
    base_frame =  robot_urdf->getRoot()->name;

    // Joint names from URDF without floating base
    std::vector<std::string> joint_names_urdf = URDFTools::jointNamesFromURDF(robot_urdf);

    // Add floating base
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(cfg.floating_base){
        joint_names_floating_base = URDFTools::addFloatingBaseToURDF(robot_urdf);
        world_frame = robot_urdf->getRoot()->name;
    }

    // Read Joint Limits
    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    // Parse KDL Tree
    if(!kdl_parser::treeFromUrdfModel(*robot_urdf, full_tree)){
        LOG_ERROR("Unable to load KDL Tree from file %s", cfg.file.c_str());
        return false;
    }

    joint_names = independent_joint_names = joint_names_floating_base + joint_names_urdf;
    actuated_joint_names = joint_names_urdf;
    joint_state.elements.resize(joint_names.size());
    joint_state.names = joint_names;

    // 2. Verify consistency of URDF and config

    // All contact point have to be a valid link in the robot URDF
    for(auto c : cfg.contact_points.names){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }

    // 3. Create data structures

    q.resize(noOfJoints());
    qd.resize(noOfJoints());
    qdd.resize(noOfJoints());
    tau.resize(noOfJoints());
    zero.resize(noOfJoints());
    zero.data.setZero();
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
    createChain(full_tree, root_frame, tip_frame);
}

void RobotModelKDL::createChain(const KDL::Tree& tree, const std::string &root_frame, const std::string &tip_frame){
    KDL::Chain chain;
    if(!tree.getChain(root_frame, tip_frame, chain)){
        LOG_ERROR("Unable to extract kinematics chain from %s to %s from KDL tree", root_frame.c_str(), tip_frame.c_str());
        throw std::invalid_argument("Invalid robot model config");
    }

    const std::string chain_id = chainID(root_frame, tip_frame);

    KinematicChainKDLPtr kin_chain = std::make_shared<KinematicChainKDL>(chain, root_frame, tip_frame);
    kin_chain->update(q,qd,qdd,joint_idx_map_kdl);
    kdl_chain_map[chain_id] = kin_chain;

    LOG_INFO_S<<"Added chain "<<root_frame<<" --> "<<tip_frame<<std::endl;
}

void RobotModelKDL::update(const base::samples::Joints& joint_state_in,
                           const base::samples::RigidBodyStateSE3& _floating_base_state){

    if(joint_state_in.elements.size() != joint_state_in.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state_in.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    for(auto n : actuated_joint_names)
        joint_state[n] = joint_state_in[n];
    joint_state.time = joint_state_in.time;

    // Update floating base if available
    if(has_floating_base){
        if(!_floating_base_state.hasValidPose() ||
           !_floating_base_state.hasValidTwist() ||
           !_floating_base_state.hasValidAcceleration()){
           LOG_ERROR("Invalid status of floating base given! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion)");
           throw std::runtime_error("Invalid floating base status");
        }
        if(_floating_base_state.time.isNull()){
            LOG_ERROR("Floating base state does not have a valid timestamp. Or do we have 1970?");
            throw std::runtime_error("Invalid call to update()");
        }
        floating_base_state = _floating_base_state;
        base::Vector3d euler = floating_base_state.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        for(int i = 0; i < 3; i++){
            q(i)   = joint_state[joint_names_floating_base[i]].position     = floating_base_state.pose.position(i);
            qd(i)  = joint_state[joint_names_floating_base[i]].speed        = floating_base_state.twist.linear(i);
            qdd(i) = joint_state[joint_names_floating_base[i]].acceleration = floating_base_state.acceleration.linear(i);

            q(i+3)   = joint_state[joint_names_floating_base[i+3]].position     = euler(i);
            qd(i+3)  = joint_state[joint_names_floating_base[i+3]].speed        = floating_base_state.twist.angular(i);
            qdd(i+3) = joint_state[joint_names_floating_base[i+3]].acceleration = floating_base_state.acceleration.angular(i);
        }
        if(floating_base_state.time > joint_state.time)
            joint_state.time = floating_base_state.time;
    }

    // Update actuated joints
    for(size_t i = 0; i < noOfActuatedJoints(); i++){
        const std::string &name = actuated_joint_names[i];
        if(!hasJoint(name)){
            LOG_ERROR_S << "Joint " << name << " is a non-fixed joint in the KDL Tree, but it is not in the joint state vector."
                        << "You should either set the joint to 'fixed' in your URDF file or provide a valid joint state for it" << std::endl;
            throw std::runtime_error("Incomplete Joint State");
        }
        const base::JointState &state = joint_state_in[name];
        uint idx = joint_idx_map_kdl[name];
        q(idx) = state.position;
        qd(idx) = state.speed;
        qdd(idx) = state.acceleration;
    }

    for(auto c : kdl_chain_map)
        c.second->update(q,qd,qdd,joint_idx_map_kdl);
}

void RobotModelKDL::systemState(base::VectorXd &_q, base::VectorXd &_qd, base::VectorXd &_qdd){
    _q.resize(noOfJoints());
    _qd.resize(noOfJoints());
    _qdd.resize(noOfJoints());

    for(int i = 0; i < noOfJoints(); i++){
        const std::string& name = jointNames()[i];
        uint idx = joint_idx_map_kdl[name];
        _q[i] = q(idx);
        _qd[i] = qd(idx);
        _qdd[i] = qdd(idx);
    }
}

const base::samples::RigidBodyStateSE3 &RobotModelKDL::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chainID(root_frame, tip_frame)];
    kdl_chain->calculateForwardKinematics();
    rbs = kdl_chain->rigidBodyState();

    return rbs;
}

const base::MatrixXd& RobotModelKDL::spaceJacobian(const std::string &root_frame, const std::string &tip_frame){
    return spaceJacobianFromTree(full_tree, root_frame, tip_frame);
}

const base::MatrixXd& RobotModelKDL::spaceJacobianFromTree(const KDL::Tree& tree, const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // Create chain if it does not exist
    const std::string chain_id = chainID(root_frame, tip_frame);
    if(kdl_chain_map.count(chain_id) == 0)
        createChain(tree, root_frame, tip_frame);

    KinematicChainKDLPtr kdl_chain = kdl_chain_map[chain_id];
    kdl_chain->calculateSpaceJacobian();

    space_jac_map[chain_id].resize(6,noOfJoints());
    space_jac_map[chain_id].setZero();
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        space_jac_map[chain_id].col(idx) = kdl_chain->space_jacobian.data.col(j);
    }
    return space_jac_map[chain_id];
}

const base::MatrixXd& RobotModelKDL::bodyJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
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
    body_jac_map[chain_id].setZero();
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        body_jac_map[chain_id].col(idx) = kdl_chain->body_jacobian.data.col(j);
    }
    return body_jac_map[chain_id];
}


const base::MatrixXd &RobotModelKDL::comJacobian(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    com_jac = base::MatrixXd::Zero(3, noOfJoints());

    // create a copy in which, for each segment with mass, COG frames are added
    KDL::Tree tree_cog_frames = full_tree;

    double totalMass = 0;
    std::map<std::string, double> mass_map;

    std::vector<std::string> COGSegmentNames;
    for(auto& segment_it : full_tree.getSegments())
    {
        auto segment = segment_it.second.segment;
        std::string segmentName = segment.getName();
        // Skip root segment! To get consistent results with RBDL, Pinocchio, etc. we have to start with the first child of Root segment! Why?
        if(segmentName == full_tree.getRootSegment()->second.segment.getName())
            continue;
        std::string segmentNameCOG = segmentName + "_COG";
        
        KDL::Vector segmentCOG = segment.getInertia().getCOG();
        double segmentMass = segment.getInertia().getMass();

        if(segmentMass == 0.0)
            continue;

        KDL::Frame frameCOG(KDL::Rotation(), segmentCOG);
        tree_cog_frames.addSegment(KDL::Segment(segmentNameCOG, KDL::Joint(KDL::Joint::Fixed), frameCOG), segmentName);

        mass_map[segmentNameCOG] = segmentMass;
        totalMass += segmentMass;

        COGSegmentNames.push_back(segmentNameCOG);
    }
    
    // compute com jacobian as (mass) weighted average over COG frame jacobians
    for(const auto& segment_name : COGSegmentNames)
        com_jac += (mass_map[segment_name] / totalMass) *
            spaceJacobianFromTree(tree_cog_frames, tree_cog_frames.getRootSegment()->second.segment.getName(), segment_name).topRows<3>();

    return com_jac;
}

const base::MatrixXd &RobotModelKDL::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
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
    jac_dot_map[chain_id].setZero();
    for(uint j = 0; j < kdl_chain->joint_names.size(); j++){
        int idx = jointIndex(kdl_chain->joint_names[j]);
        jac_dot_map[chain_id].col(idx) = kdl_chain->jacobian_dot.data.col(j);
    }
    return jac_dot_map[chain_id];
}

const base::Acceleration &RobotModelKDL::spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame){
    qdot_tmp.resize(noOfJoints());
    tmp_acc.resize(6);
    for(int i = 0; i < joint_names.size(); i++)
        qdot_tmp[i] = qd(joint_idx_map_kdl[joint_names[i]]);
    tmp_acc = jacobianDot(root_frame, tip_frame)*qdot_tmp;
    spatial_acc_bias = base::Acceleration(tmp_acc.segment(0,3), tmp_acc.segment(3,3));
    return spatial_acc_bias;
}

const base::VectorXd &RobotModelKDL::biasForces(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    // Use ID solver with zero joint accelerations and zero external wrenches to get bias forces/torques
    KDL::TreeIdSolver_RNE solver(full_tree, KDL::Vector(gravity(0), gravity(1), gravity(2)));
    solver.CartToJnt(q, qd, zero, std::map<std::string,KDL::Wrench>(), tau);

    for(uint i = 0; i < joint_names.size(); i++){
        const std::string &name = joint_names[i];
        uint idx = joint_idx_map_kdl[name];
        bias_forces[i] = tau(idx);
    }
    return bias_forces;
}

const base::MatrixXd& RobotModelKDL::jointSpaceInertiaMatrix(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jacobianDot()");
    }

    joint_space_inertia_mat.setZero();

    // Use ID solver with zero bias and external wrenches to compute joint space inertia matrix column by column.
    // TODO: Switch to more efficient method!
    KDL::TreeIdSolver_RNE solver(full_tree, KDL::Vector::Zero());
    for(uint i = 0; i < joint_names.size(); i++){
        const std::string& name = joint_names[i];
        qdd.data.setZero();
        qdd(joint_idx_map_kdl[name]) = 1;
        int ret = solver.CartToJnt(q, zero, qdd, std::map<std::string,KDL::Wrench>(), tau);
        if(ret != 0)
            throw(std::runtime_error("Unable to compute Tree Inverse Dynamics in joint space inertia matrix computation. Error Code is " + std::to_string(ret)));
        for(int j = 0; j < joint_names.size(); j++){
            const std::string& _name = joint_names[j];
            joint_space_inertia_mat(j, i) = tau(joint_idx_map_kdl[_name]);
        }
    }
    return joint_space_inertia_mat;
}


void RobotModelKDL::recursiveCOM(const KDL::SegmentMap::const_iterator& currentSegment, const KDL::Frame& frame, double& mass, KDL::Vector& cog)
{
    double jointPosition = 0.0; // Joint position of the current joint
#ifdef KDL_USE_NEW_TREE_INTERFACE
    KDL::Segment segment = currentSegment->second->segment;
#else
    KDL::Segment segment = currentSegment->second.segment;
#endif
    // If the segment has a real joint, get the joint position
    if( segment.getJoint().getType() != KDL::Joint::None ) {        
        const std::string name = segment.getJoint().getName();
        if(joint_idx_map_kdl.count(name)==0){
            LOG_ERROR_S  << "recursiveCOM: KDL tree contains joint " << name << " but this joint does not exist in joint idx map" << std::endl;
            throw std::runtime_error("Problem with model configuration");
        }
        uint idx = joint_idx_map_kdl[name];
        jointPosition = q(idx);
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
        recursiveCOM( *childSegment, currentFrame, mass, cog );
    }
}

const base::samples::RigidBodyStateSE3& RobotModelKDL::centerOfMass(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to centerOfMass()");
    }

    double mass = 0.0; // to get the total mass
    KDL::Frame frame = KDL::Frame::Identity(); // Transformation of the last frame

    // Computes the COG of the complete robot
    KDL::Vector cog_pos;
    // To get consistent results with RBDL, Pinocchio, etc. we have to start with the first child of Root link! Why?
    recursiveCOM( full_tree.getRootSegment()->second.children[0], frame, mass, cog_pos );

    // Returns the COG
    com_rbs.frame_id = world_frame;
    com_rbs.pose.position = base::Vector3d( cog_pos.x(), cog_pos.y(), cog_pos.z() ) / mass;
    com_rbs.pose.orientation.setIdentity();
    com_rbs.time = joint_state.time;
    return com_rbs;
}

void RobotModelKDL::computeInverseDynamics(base::commands::Joints &solver_output){
    if(joint_state.time.isNull()){
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
    int ret = solver.CartToJnt(q, qd, qdd, w_map, tau);
    if(ret != 0)
        throw(std::runtime_error("Unable to compute Tree Inverse Dynamics. Error Code is " + std::to_string(ret)));

    for(uint i = 0; i < noOfActuatedJoints(); i++){
        const std::string& name = actuated_joint_names[i];
        solver_output[name].effort = tau(joint_idx_map_kdl[name]);
    }
}
}
