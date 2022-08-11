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

    robot_model_config = cfg;
    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }
    base_frame = robot_urdf->getRoot()->name;

    // Add floating base
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(has_floating_base){
        joint_names_floating_base = URDFTools::addFloatingBaseToURDF(robot_urdf);
        world_frame = robot_urdf->getRoot()->name;
    }

    // Read Joint Limits
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

    joint_state.names = hyrodyn.jointnames_spanningtree;
    joint_state.elements.resize(hyrodyn.jointnames_spanningtree.size());

    joint_names = joint_names_floating_base + hyrodyn.jointnames_active;
    actuated_joint_names = hyrodyn.jointnames_active;
    independent_joint_names = hyrodyn.jointnames_independent;

    // 2. Verify consistency of URDF and configurdf_model

    // All contact point have to be a valid link in the robot URDF
    for(auto c : cfg.contact_points.names){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }

    // 3. Create data structures

    active_contacts = cfg.contact_points;
    joint_space_inertia_mat.resize(noOfJoints(), noOfJoints());
    bias_forces.resize(noOfJoints());
    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < hyrodyn.jointnames_active.size(); i++)
        selection_matrix(i, jointIndex(hyrodyn.jointnames_active[i])) = 1.0;

    LOG_DEBUG("------------------- WBC RobotModelHyrodyn -----------------");
    LOG_DEBUG_S << "Robot Name " << robot_urdf->getName() << std::endl;
    LOG_DEBUG_S << "Floating base robot: " << hyrodyn.floating_base_robot << std::endl;
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

    if(has_floating_base){

        floating_base_state = _floating_base_state;

        // Transformation from fb body linear acceleration to fb joint linear acceleration
        // look at RobotModelRBDL for description
        Eigen::Matrix3d fb_rot = _floating_base_state.pose.orientation.toRotationMatrix();
        base::Twist fb_twist = _floating_base_state.twist;
        base::Acceleration fb_acc = _floating_base_state.acceleration;

        Eigen::VectorXd spherical_j_vel(6);
        spherical_j_vel << fb_twist.angular, Eigen::Vector3d::Zero();
        Eigen::VectorXd spherical_b_vel(6);
        spherical_b_vel << fb_twist.angular, fb_rot.transpose() * fb_twist.linear;
        Eigen::VectorXd fb_spherical_cross = crossm(spherical_b_vel, spherical_j_vel);
        fb_acc.linear = fb_acc.linear - fb_rot * fb_spherical_cross.tail<3>(); // remove cross contribution from linear acc s(in world coordinates as RBDL want)

        hyrodyn.floating_robot_pose.segment(0,3) = _floating_base_state.pose.position;
        hyrodyn.floating_robot_pose[3] = _floating_base_state.pose.orientation.x();
        hyrodyn.floating_robot_pose[4] = _floating_base_state.pose.orientation.y();
        hyrodyn.floating_robot_pose[5] = _floating_base_state.pose.orientation.z();
        hyrodyn.floating_robot_pose[6] = _floating_base_state.pose.orientation.w();
        hyrodyn.floating_robot_twist.segment(0,3) = _floating_base_state.twist.angular;
        hyrodyn.floating_robot_twist.segment(3,3) = _floating_base_state.twist.linear;
        hyrodyn.floating_robot_accn.segment(0,3) = fb_acc.angular;
        hyrodyn.floating_robot_accn.segment(3,3) = fb_acc.linear;

        for( unsigned int i = 6; i < hyrodyn.jointnames_independent.size(); ++i){
            const std::string& name =  hyrodyn.jointnames_independent[i];
            try{
                hyrodyn.y_robot[i-6]   = joint_state_in[name].position;
                hyrodyn.yd_robot[i-6]  = joint_state_in[name].speed;
                hyrodyn.ydd_robot[i-6] = joint_state_in[name].acceleration;
            }
            catch(base::samples::Joints::InvalidName e){
                LOG_ERROR_S << "Joint " << name << " is in independent joints of Hyrodyn model, but it is not given in joint state vector" << std::endl;
                throw e;
            }
        }
        hyrodyn.update_all_independent_coordinates();
    }
    else{
        for( unsigned int i = 0; i < hyrodyn.jointnames_independent.size(); ++i){
            const std::string& name =  hyrodyn.jointnames_independent[i];
            try{
                hyrodyn.y[i]   = joint_state_in[name].position;
                hyrodyn.yd[i]  = joint_state_in[name].speed;
                hyrodyn.ydd[i] = joint_state_in[name].acceleration;
            }
            catch(base::samples::Joints::InvalidName e){
                LOG_ERROR_S << "Joint " << name << " is in independent joints of Hyrodyn model, but it is not given in joint state vector" << std::endl;
                throw e;
            }
        }
    }

    // Compute system state
    hyrodyn.calculate_system_state();

    for(size_t i = 0; i < hyrodyn.jointnames_spanningtree.size(); i++){
        const std::string &name = hyrodyn.jointnames_spanningtree[i];
        joint_state[name].position = hyrodyn.Q[i];
        joint_state[name].speed = hyrodyn.QDot[i];
        joint_state[name].acceleration = hyrodyn.QDDot[i];
    }
    joint_state.time = joint_state_in.time;
}

void RobotModelHyrodyn::systemState(base::VectorXd &_q, base::VectorXd &_qd, base::VectorXd &_qdd){
    _q = hyrodyn.y;
    _qd = hyrodyn.yd;
    _qdd = hyrodyn.ydd;
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

    std::string chain_id = chainID(root_frame,tip_frame);

    space_jac_map[chain_id].resize(6,noOfJoints());
    space_jac_map[chain_id].setZero();
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_space_jacobian_actuation_space_including_floatingbase(tip_frame);
        uint n_cols = hyrodyn.Jsufb.cols();
        space_jac_map[chain_id].block(0,0,3,n_cols) = hyrodyn.Jsufb.block(3,0,3,n_cols);
        space_jac_map[chain_id].block(3,0,3,n_cols) = hyrodyn.Jsufb.block(0,0,3,n_cols);
    }else{
        hyrodyn.calculate_space_jacobian_actuation_space(tip_frame);
        uint n_cols = hyrodyn.Jsu.cols();
        space_jac_map[chain_id].block(0,0,3,n_cols) = hyrodyn.Jsu.block(3,0,3,n_cols);
        space_jac_map[chain_id].block(3,0,3,n_cols) = hyrodyn.Jsu.block(0,0,3,n_cols);
    }

    return space_jac_map[chain_id];
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

    std::string chain_id = chainID(root_frame,tip_frame);

    body_jac_map[chain_id].resize(6,noOfJoints());
    body_jac_map[chain_id].setZero();
    if(hyrodyn.floating_base_robot){
        hyrodyn.calculate_body_jacobian_actuation_space_including_floatingbase(tip_frame);
        uint n_cols = hyrodyn.Jbufb.cols();
        body_jac_map[chain_id].block(0,0,3,n_cols) = hyrodyn.Jbufb.block(3,0,3,n_cols);
        body_jac_map[chain_id].block(3,0,3,n_cols) = hyrodyn.Jbufb.block(0,0,3,n_cols);
    }
    else{
        hyrodyn.calculate_body_jacobian_actuation_space(tip_frame);
        uint n_cols = hyrodyn.Jbu.cols();
        body_jac_map[chain_id].block(0,0,3,n_cols) = hyrodyn.Jbu.block(3,0,3,n_cols);
        body_jac_map[chain_id].block(3,0,3,n_cols) = hyrodyn.Jbu.block(0,0,3,n_cols);
    }

    return body_jac_map[chain_id];
}

const base::MatrixXd &RobotModelHyrodyn::comJacobian(){
    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelHyrodyn: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to comJacobian()");
    }

    hyrodyn.calculate_com_jacobian();
    com_jac.resize(3,noOfJoints());
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
