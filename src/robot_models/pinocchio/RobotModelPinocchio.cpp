#include "RobotModelPinocchio.hpp"
#include <base-logging/Logging.hpp>
#include "../../tools/URDFTools.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace wbc{

RobotModelPinocchio::RobotModelPinocchio(){

}

RobotModelPinocchio::~RobotModelPinocchio(){
}

void RobotModelPinocchio::clear(){

    RobotModel::clear();
    data.reset();
    model = pinocchio::Model();
}

bool RobotModelPinocchio::configure(const RobotModelConfig& cfg){

    clear();

    // 1. Load Robot Model

    robot_model_config = cfg;

    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }

    // Blacklist not required joints
    if(!URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist))
        return false;

    // Add floating base
    has_floating_base = cfg.floating_base;

    base_frame = robot_urdf->getRoot()->name;

    try{
        if(has_floating_base){
            pinocchio::urdf::buildModel(robot_urdf,pinocchio::JointModelFreeFlyer(), model);
            world_frame = cfg.world_frame_id;
        }
        else{
            pinocchio::urdf::buildModel(robot_urdf, model);
            world_frame = base_frame;
        }
    }
    catch(std::invalid_argument e){
        LOG_ERROR_S << "RobotModelPinocchio: Failed to load urdf model from " << cfg.file <<std::endl;
        return false;
    }
    data = std::make_shared<pinocchio::Data>(model);

    joint_names = model.names;
    joint_names.erase(joint_names.begin()); // Erase global joint 'universe' which is added by Pinocchio
    if(has_floating_base)
        joint_names.erase(joint_names.begin()); // Erase 'floating_base' root joint
    actuated_joint_names = independent_joint_names = joint_names;

    // Joint order in q,qd,qdd:
    //
    // Floating base:
    // q:   x,y,z,qw,qx,qy,qz,joints_q,         Size: 7 + joint_names.size()
    // qd:  vx,vy,vz,wx,wx,wy,wz,joints_dq,     Size: 6 + joint_names.size()
    // qdd: dvx,dvy,dvz,dwx,dwy,dwz,joints_ddq, Size: 6 + joint_names.size()
    //
    // Fixed base:
    // q:   joint_names_q,   Size: joint_names.size()
    // qd:  joint_names_dq,  Size: joint_names.size()
    // qdd: joint_names_ddq, Size: joint_names.size()
    q.resize(model.nq);
    qd.resize(model.nv);
    qdd.resize(model.nv);

    joint_state.resize(noOfJoints());
    joint_state.names = jointNames();

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    LOG_DEBUG("------------------- WBC RobotModelPinocchio -----------------");
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

void RobotModelPinocchio::update(const base::samples::Joints& joint_state_in,
                                 const base::samples::RigidBodyStateSE3& floating_base_state_in){
    if(joint_state_in.elements.size() != joint_state_in.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state_in.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(has_floating_base){

        floating_base_state = floating_base_state_in;

        // Transform the linear parts of spatial velocity and acceleration to local coordinates to
        // match the pinocchio convention (see here https://sites.google.com/site/xinsongyan/researches/rbdl-pinnochio)
        // TODO: Is this correct?
        base::Pose tf;
        tf.fromTransform(floating_base_state.pose.toTransform().inverse());
        base::Twist twist_linear_local;
        base::Acceleration acc_linear_local;
        twist_linear_local.linear = tf.orientation.inverse() * floating_base_state.twist.linear;
        acc_linear_local.linear   = tf.orientation.inverse() * floating_base_state.acceleration.linear;

        for(int i = 0; i < 3; i++){
            q[i] = floating_base_state.pose.position[i];
            qd[i] = twist_linear_local.linear[i];
            qd[i+3] = floating_base_state.twist.angular[i];
            qdd[i] = acc_linear_local.linear[i];
            qdd[i+3] = floating_base_state.acceleration.angular[i];
        }
        q[3] = floating_base_state.pose.orientation.x();
        q[4] = floating_base_state.pose.orientation.y();
        q[5] = floating_base_state.pose.orientation.z();
        q[6] = floating_base_state.pose.orientation.w();
    }

    for(size_t i = 0; i < noOfActuatedJoints(); i++){
        const std::string& name = actuated_joint_names[i];
        std::size_t idx;
        try{
            idx = joint_state_in.mapNameToIndex(name);
        }
        catch(base::samples::Joints::InvalidName e){
            LOG_ERROR_S<<"Robot model contains joint "<<name<<" but this joint is not in joint state vector"<<std::endl;
            throw e;
        }
        joint_state[name] = joint_state_in[idx];
    }
    joint_state.time = joint_state_in.time;

    // Joint IDs in Pinocchio:
    //
    // Floating base
    // universe,root_joint,joint_names
    //
    // Fixed base
    // universe,joint_names
    for(auto name : joint_names){
        base::JointState state = joint_state[name];
        if(has_floating_base){
            // Subtract 2 due to universe & root_joint
            q[model.getJointId(name)-2+7]   = state.position;     // first 7 elements in q are floating base pose
            qd[model.getJointId(name)-2+6]  = state.speed;        // first 6 elements in q are floating base twist
            qdd[model.getJointId(name)-2+6] = state.acceleration; // first 6 elements in q are floating base acceleration
        }
        else{
            q[model.getJointId(name)-1]   = state.position;
            qd[model.getJointId(name)-1]  = state.speed;
            qdd[model.getJointId(name)-1] = state.acceleration;
        }
    }
}

const base::samples::RigidBodyStateSE3 &RobotModelPinocchio::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested Forward kinematics computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but the pinocchio robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    if(!hasLink(tip_frame)){
        LOG_ERROR_S<<"Requested Forward kinematics for tip frame "<<tip_frame<<" but this frame does not exist in robot model"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }

    pinocchio::forwardKinematics(model,*data,q,qd,qdd);
    pinocchio::updateFramePlacement(model,*data,model.getFrameId(tip_frame));

    rbs.time = joint_state.time;
    rbs.frame_id = root_frame;
    rbs.pose.position = data->oMf[model.getFrameId(tip_frame)].translation();
    rbs.pose.orientation = base::Quaterniond(data->oMf[model.getFrameId(tip_frame)].rotation());
    // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
    // but with axes aligned with the frame of the Universe. This a MIXED representation betwenn the LOCAL and the WORLD conventions.
    rbs.twist.linear = pinocchio::getFrameVelocity(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.twist.angular = pinocchio::getFrameVelocity(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();
    rbs.acceleration.linear = pinocchio::getFrameAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.acceleration.angular = pinocchio::getFrameAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();
    return rbs;
}

const base::MatrixXd &RobotModelPinocchio::spaceJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested space Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but the pinocchio robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    if(!hasLink(tip_frame)){
        LOG_ERROR_S<<"Requested space Jacobian for tip frame "<<tip_frame<<" but this frame does not exist in robot model"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }
    std::string chain_id = chainID(root_frame, tip_frame);
    space_jac_map[chain_id].resize(6,model.nv);
    space_jac_map[chain_id].setZero();
    pinocchio::computeFrameJacobian(model, *data, q, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED, space_jac_map[chain_id]);

    return space_jac_map[chain_id];
}

const base::MatrixXd &RobotModelPinocchio::bodyJacobian(const std::string &root_frame, const std::string &tip_frame){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested space Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but the pinocchio robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    if(!hasLink(tip_frame)){
        LOG_ERROR_S<<"Requested space Jacobian for tip frame "<<tip_frame<<" but this frame does not exist in robot model"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }
    std::string chain_id = chainID(root_frame, tip_frame);
    body_jac_map[chain_id].resize(6,model.nv);
    body_jac_map[chain_id].setZero();
    pinocchio::computeFrameJacobian(model, *data, q, model.getFrameId(tip_frame), pinocchio::LOCAL, body_jac_map[chain_id]);

    return body_jac_map[chain_id];
}

const base::MatrixXd &RobotModelPinocchio::comJacobian(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error("Invalid call to comJacobian()");
    }

    pinocchio::jacobianCenterOfMass(model, *data, q);
    com_jac.resize(3,noOfJoints());
    com_jac = data->Jcom;
    return com_jac;
}

const base::Acceleration &RobotModelPinocchio::spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame){
    pinocchio::forwardKinematics(model,*data,q,qd,base::VectorXd::Zero(model.nv));
    spatial_acc_bias.linear = pinocchio::getFrameClassicalAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    spatial_acc_bias.angular = pinocchio::getFrameClassicalAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();
    return spatial_acc_bias;
}

const base::MatrixXd &RobotModelPinocchio::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

    throw std::runtime_error("Not implemented: jacobianDot has not been implemented for RobotModelPinocchio");
}

const base::MatrixXd &RobotModelPinocchio::jointSpaceInertiaMatrix(){   

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jointSpaceInertiaMatrix()");
    }

    pinocchio::crba(model, *data, q);
    joint_space_inertia_mat = data->M;
    // copy upper right triangular part to lower left triangular part (they are symmetric), as pinocchio only computes the former
    joint_space_inertia_mat.triangularView<Eigen::StrictlyLower>() = joint_space_inertia_mat.transpose().triangularView<Eigen::StrictlyLower>();
    return joint_space_inertia_mat;
}

const base::VectorXd &RobotModelPinocchio::biasForces(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to biasForces()");
    }

    pinocchio::nonLinearEffects(model, *data, q, qd);
    bias_forces = data->nle;
    return bias_forces;
}

const base::samples::RigidBodyStateSE3& RobotModelPinocchio::centerOfMass(){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to centerOfMass()");
    }

    pinocchio::centerOfMass(model, *data, q, qd, qdd);
    com_rbs.pose.position       = data->com[0];
    com_rbs.twist.linear        = data->vcom[0];
    com_rbs.acceleration.linear = data->acom[0];
    com_rbs.pose.orientation.setIdentity();
    com_rbs.twist.angular.setZero();
    com_rbs.acceleration.angular.setZero();
    com_rbs.time = joint_state.time;
    com_rbs.frame_id = world_frame;
    return com_rbs;
}

void RobotModelPinocchio::computeInverseDynamics(base::commands::Joints &solver_output){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to centerOfMass()");
    }

    // TODO: Add external wrenches here
    pinocchio::rnea(model, *data, q, qd, qdd);

    for(uint i = 0; i < noOfJoints(); i++){
        const std::string &name = jointNames()[i];
        solver_output[name].effort = data->tau[i];
    }
}

}
