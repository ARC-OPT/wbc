#include "RobotModelHyrodyn.hpp"
#include <base-logging/Logging.hpp>
#include <urdf_parser/urdf_parser.h>
#include <tools/URDFTools.hpp>

namespace wbc{

RobotModelRegistry<RobotModelHyrodyn> RobotModelHyrodyn::reg("hyrodyn");

RobotModelHyrodyn::RobotModelHyrodyn(){
}

RobotModelHyrodyn::~RobotModelHyrodyn(){
}

void RobotModelHyrodyn::clear(){

    RobotModel::clear();

    hyrodyn = hyrodyn::RobotModel_HyRoDyn();
}

bool RobotModelHyrodyn::configure(const RobotModelConfig& cfg){

    clear();

    // 1. Load Robot Model

    if(!cfg.joint_names.empty())
        LOG_WARN("Configured joint names will be ignored! The Hyrodyn based model will get the joint names from submechanism file");
    if(!cfg.actuated_joint_names.empty())
        LOG_WARN("Configured actuated joint names will be ignored! The Hyrodyn based model will get the actuated joint names from submechanism file");

    robot_model_config = cfg;

    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }
    base_frame = robot_urdf->getRoot()->name;

    // Blacklist not required joints
    if(!URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist))
        return false;

    // Add floating base
    world_frame = base_frame;
    has_floating_base = cfg.floating_base;
    if(has_floating_base){
        joint_names_floating_base = URDFTools::addFloatingBaseToURDF(robot_urdf, cfg.world_frame_id);
        world_frame = robot_urdf->getRoot()->name;
    }

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    TiXmlDocument *doc = urdf::exportURDF(robot_urdf);
    std::string robot_urdf_file = "/tmp/floating_base_model.urdf";
    doc->SaveFile(robot_urdf_file);
    try{
        hyrodyn.load_robotmodel(robot_urdf_file, cfg.submechanism_file);
    }
    catch(std::exception e){
        LOG_ERROR_S << "Failed to load hyrodyn model from URDF " << robot_urdf_file <<
                       " and submechanism file " << cfg.submechanism_file << std::endl;
        return false;
    }

    joint_state.names =hyrodyn.jointnames_spanningtree;
    joint_state.elements.resize(hyrodyn.jointnames_spanningtree.size());

    joint_names = joint_names_floating_base + hyrodyn.jointnames_active;
    actuated_joint_names = hyrodyn.jointnames_active;
    independent_joint_names = joint_names_floating_base + hyrodyn.jointnames_independent;

    // 2. Verify consistency of URDF and config

    // This is mostly being done internally in hyrodyn

    // All contact point have to be a valid link in the robot URDF
    for(auto c : cfg.contact_points.names){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }

    // 3. Set initial floating base state

    if(hyrodyn.floating_base_robot){
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
                updateFloatingBase(rbs, joint_names_floating_base, joint_state);
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

    jac.resize(6,noOfJoints());
    jac.setConstant(std::numeric_limits<double>::quiet_NaN());
    com_jac.resize(3,noOfJoints());
    com_jac.setConstant(std::numeric_limits<double>::quiet_NaN());
    active_contacts = cfg.contact_points;
    joint_space_inertia_mat.resize(noOfJoints(), noOfJoints());
    bias_forces.resize(noOfJoints());
    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < hyrodyn.jointnames_active.size(); i++)
        selection_matrix(i, jointIndex(hyrodyn.jointnames_active[i])) = 1.0;

    // 5. Print some debug info

    LOG_DEBUG("------------------- WBC RobotModelHyrodyn -----------------");
    LOG_DEBUG_S << "Robot Name " << robot_urdf->getName() << std::endl;
    LOG_DEBUG_S << "Floating base robot: " << hyrodyn.floating_base_robot << std::endl;
    if(hyrodyn.floating_base_robot){
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

void RobotModelHyrodyn::update(const base::samples::Joints& joint_state_in,
                               const base::samples::RigidBodyStateSE3& _floating_base_state){

    if(joint_state_in.elements.size() != joint_state_in.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state_in.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    uint start_idx = 0;
    // Update floating base if available
    if(hyrodyn.floating_base_robot){
        updateFloatingBase(_floating_base_state, joint_names_floating_base, joint_state);
        start_idx = 6;
        for(int i = 0; i < 6; i++){
            hyrodyn.y(i)   = joint_state[i].position;
            hyrodyn.yd(i)   = joint_state[i].speed;
            hyrodyn.ydd(i)   = joint_state[i].acceleration;
        }
    }

    // Update independent joints. This assumes that joints 0..5 are the floating base joints
    for( unsigned int i = start_idx; i < hyrodyn.jointnames_independent.size(); ++i){
        const std::string& name =  hyrodyn.jointnames_independent[i];
        try{
            hyrodyn.y[i] = joint_state_in[name].position;
            hyrodyn.yd[i] = joint_state_in[name].speed;
            hyrodyn.ydd[i] = joint_state_in[name].acceleration;
        }
        catch(base::samples::Joints::InvalidName e){
            LOG_ERROR_S << "Joint " << name << " is in independent joints of Hyrodyn model, but it is not given in joint state vector" << std::endl;
            throw e;
        }
    }

    // Compute system state
    hyrodyn.calculate_system_state();

    for(size_t i = 0; i < hyrodyn.jointnames_spanningtree.size(); i++){
        const std::string &name = hyrodyn.jointnames_spanningtree[i];
        joint_state[name].position = hyrodyn.Q[i];
        joint_state[name].speed = hyrodyn.QDot[i];
        joint_state[name].acceleration = hyrodyn.QDDot[i];
        //joint_state[name].effort = hyrodyn.Tau_spanningtree[i]; // It seems Tau_spanningtree is currently not being computed by hyrodyn
    }
    joint_state.time = joint_state_in.time;
}

const base::samples::RigidBodyStateSE3 &RobotModelHyrodyn::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested Forward kinematics computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    hyrodyn.calculate_forward_kinematics(tip_frame);
    rbs.pose.position        = hyrodyn.pose.segment(0,3);
    rbs.pose.orientation     = base::Quaterniond(hyrodyn.pose[6],hyrodyn.pose[3],hyrodyn.pose[4],hyrodyn.pose[5]);
    rbs.twist.linear         = hyrodyn.twist.segment(3,3);
    rbs.twist.angular        = hyrodyn.twist.segment(0,3);
    rbs.acceleration.linear  = hyrodyn.spatial_acceleration.segment(3,3);
    rbs.acceleration.angular = hyrodyn.spatial_acceleration.segment(0,3);//
    rbs.time                 = joint_state.time;
    rbs.frame_id             = tip_frame;

    return rbs;
}

const base::MatrixXd &RobotModelHyrodyn::spaceJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to spaceJacobian()");
    }

    if(!hasLink(root_frame)){
        LOG_ERROR_S << "Request jacobian for " << root_frame << " -> " << tip_frame << " but link " << root_frame << " does not exist in robot model" << std::endl;
        throw std::runtime_error("Invalid call to spaceJacobian()");
    }

    if(!hasLink(tip_frame)){
        LOG_ERROR_S << "Request jacobian for " << root_frame << " -> " << tip_frame << " but link " << tip_frame << " does not exist in robot model" << std::endl;
        throw std::runtime_error("Invalid call to spaceJacobian()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    jac.setZero();
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_space_jacobian_actuation_space_including_floatingbase(tip_frame);
        uint n_cols = hyrodyn.Jsufb.cols();
        jac.block(0,0,3,n_cols) = hyrodyn.Jsufb.block(3,0,3,n_cols);
        jac.block(3,0,3,n_cols) = hyrodyn.Jsufb.block(0,0,3,n_cols);
    }else{
        hyrodyn.calculate_space_jacobian_actuation_space(tip_frame);
        uint n_cols = hyrodyn.Jsu.cols();
        jac.block(0,0,3,n_cols) = hyrodyn.Jsu.block(3,0,3,n_cols);
        jac.block(3,0,3,n_cols) = hyrodyn.Jsu.block(0,0,3,n_cols);
    }

    return jac;
}

const base::MatrixXd &RobotModelHyrodyn::bodyJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to bodyJacobian()");
    }

    if(!hasLink(root_frame)){
        LOG_ERROR_S << "Request jacobian for " << root_frame << " -> " << tip_frame << " but link " << root_frame << " does not exist in robot model" << std::endl;
        throw std::runtime_error("Invalid call to bodyJacobian()");
    }

    if(!hasLink(tip_frame)){
        LOG_ERROR_S << "Request jacobian for " << root_frame << " -> " << tip_frame << " but link " << tip_frame << " does not exist in robot model" << std::endl;
        throw std::runtime_error("Invalid call to bodyJacobian()");
    }


    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    jac.setZero();
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_body_jacobian_actuation_space_including_floatingbase(tip_frame);
        uint n_cols = hyrodyn.Jbufb.cols();
        jac.block(0,0,3,n_cols) = hyrodyn.Jbufb.block(3,0,3,n_cols);
        jac.block(3,0,3,n_cols) = hyrodyn.Jbufb.block(0,0,3,n_cols);
    }
    else{
        hyrodyn.calculate_body_jacobian_actuation_space(tip_frame);
        uint n_cols = hyrodyn.Jbu.cols();
        jac.block(0,0,3,n_cols) = hyrodyn.Jbu.block(3,0,3,n_cols);
        jac.block(3,0,3,n_cols) = hyrodyn.Jbu.block(0,0,3,n_cols);
    }

    return jac;
}

const base::MatrixXd &RobotModelHyrodyn::comJacobian(){
    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to comJacobian()");
    }

    hyrodyn.calculate_com_jacobian();
    com_jac = hyrodyn.Jcom;
    return com_jac;
}

const base::MatrixXd &RobotModelHyrodyn::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

    throw std::runtime_error("Not implemented: jacobianDot has not been implemented for RobotModelHyrodyn");
}

const base::Acceleration &RobotModelHyrodyn::spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame){
    hyrodyn.calculate_spatial_acceleration_bias(tip_frame);
    spatial_acc_bias = base::Acceleration(hyrodyn.spatial_acceleration_bias.segment(3,3), hyrodyn.spatial_acceleration_bias.segment(0,3));
    return spatial_acc_bias;
}

const base::MatrixXd &RobotModelHyrodyn::jointSpaceInertiaMatrix(){
    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jointSpaceInertiaMatrix()");
    }

    // Compute joint space inertia matrix
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_mass_interia_matrix_actuation_space_including_floatingbase();
        joint_space_inertia_mat = hyrodyn.Hufb;
    }
    else{
        hyrodyn.calculate_mass_interia_matrix_actuation_space();
        joint_space_inertia_mat = hyrodyn.Hu;
    }

    return joint_space_inertia_mat;
}

const base::VectorXd &RobotModelHyrodyn::biasForces(){
    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to biasForces()");
    }

    // Compute bias forces
    hyrodyn.ydd.setZero();
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_inverse_dynamics_including_floatingbase();
        bias_forces = hyrodyn.Tau_actuated_floatingbase;
    }
    else{
        hyrodyn.calculate_inverse_dynamics();
        bias_forces = hyrodyn.Tau_actuated;
    }

    return bias_forces;
}

const base::samples::RigidBodyStateSE3& RobotModelHyrodyn::centerOfMass(){
    hyrodyn.calculate_com_properties();

    com_rbs.frame_id = world_frame;
    com_rbs.pose.position = hyrodyn.com;
    com_rbs.pose.orientation.setIdentity();
    com_rbs.twist.linear = hyrodyn.com_vel;
    com_rbs.twist.angular.setZero();
    com_rbs.acceleration.linear = hyrodyn.com_acc; // TODO: double check CoM acceleration
    com_rbs.acceleration.angular.setZero();
    com_rbs.time = joint_state.time;
    return com_rbs;
}

void RobotModelHyrodyn::computeInverseDynamics(base::commands::Joints &solver_output){
    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to computeInverseDynamics()");
    }

    for(uint i = 0; i < noOfActuatedJoints(); i++){
        const std::string &name = hyrodyn.jointnames_active[i];
        if(solver_output[name].hasPosition())
            hyrodyn.ud[i] = solver_output[name].position;
        if(solver_output[name].hasSpeed())
            hyrodyn.ud[i] = solver_output[name].speed;
        if(solver_output[name].hasAcceleration())
            hyrodyn.udd[i] = solver_output[name].acceleration;
        else
            hyrodyn.udd[i] = 0;
    }
    hyrodyn.calculate_forward_system_state();

    uint nc = contact_points.size();
    hyrodyn.wrench_interaction.resize(nc);
    hyrodyn.wrench_points = contact_points;
    hyrodyn.wrench_resolution.resize(nc);
    hyrodyn.f_ext.resize(nc);
    for(uint i = 0; i < nc; i++){
        const std::string &name = contact_points[i];
        hyrodyn.wrench_interaction[i] = true; // Resistive
        hyrodyn.wrench_resolution[i] = true; // body coordinates
        try{
            hyrodyn.f_ext[i].segment(0,3) = contact_wrenches[name].torque;
            hyrodyn.f_ext[i].segment(3,3) = contact_wrenches[name].force;
        }
        catch(base::samples::Wrenches::InvalidName e){
            LOG_ERROR_S << "Contact point " << name << " has been configured but this name is not in contact wrench vector" << std::endl;
            throw e;
        }
    }
    hyrodyn.calculate_inverse_dynamics();
    hyrodyn.calculate_inverse_statics();
    for(uint i = 0; i < hyrodyn.jointnames_active.size(); i++){
        const std::string &name = hyrodyn.jointnames_active[i];
        solver_output[name].effort = hyrodyn.Tau_actuated[i] + hyrodyn.Tau_actuated_ext[i];
    }
}

}
