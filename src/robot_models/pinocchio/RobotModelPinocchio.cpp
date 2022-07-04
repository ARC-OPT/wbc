#include "RobotModelPinocchio.hpp"
#include <base-logging/Logging.hpp>
#include "../../tools/URDFTools.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

namespace wbc{

RobotModelPinocchio::RobotModelPinocchio(){

}

RobotModelPinocchio::~RobotModelPinocchio(){
}

void RobotModelPinocchio::clear(){

    RobotModel::clear();
    data.reset();
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

    try{
        pinocchio::urdf::buildModel(robot_urdf,model);
    }
    catch(std::invalid_argument e){
        LOG_ERROR_S << "RobotModelPinocchio: Failed to load urdf model from " << cfg.file <<std::endl;
        return false;
    }
    data = std::make_shared<pinocchio::Data>(model);

    joint_names = model.names;
    joint_names.erase(joint_names.begin()); // Erase global joint 'universe' which is added by Pinocchio
    independent_joint_names = actuated_joint_names = joint_names;

    q.resize(noOfJoints());
    qd.resize(noOfJoints());
    qdd.resize(noOfJoints());

    joint_state.resize(noOfJoints());
    joint_state.names = jointNames();

    base_frame = world_frame = robot_urdf->getRoot()->name;

    // Blacklist not required joints
    if(!URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist))
        return false;

    // TODO: floating base configuration here!

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
                                 const base::samples::RigidBodyStateSE3& floating_base_state){
    if(joint_state_in.elements.size() != joint_state_in.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state_in.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
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

    for(auto name : joint_names){
        base::JointState state = joint_state[name];
        q[model.getJointId(name)-1] = state.position;
        qd[model.getJointId(name)-1] = state.speed;
        qdd[model.getJointId(name)-1] = state.acceleration;
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
    rbs.acceleration.linear = pinocchio::getFrameClassicalAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.acceleration.angular = pinocchio::getFrameClassicalAcceleration(model, *data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();
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
    jac.resize(6,model.nv);
    jac.setZero();
    pinocchio::computeFrameJacobian(model, *data, q, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED, jac);

    return jac;
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
    jac.resize(6,model.nv);
    jac.setZero();
    pinocchio::computeFrameJacobian(model, *data, q, model.getFrameId(tip_frame), pinocchio::LOCAL, jac);

    return jac;
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
    pinocchio::forwardKinematics(model,*data,q,qd,base::VectorXd::Zero(noOfJoints()));
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
