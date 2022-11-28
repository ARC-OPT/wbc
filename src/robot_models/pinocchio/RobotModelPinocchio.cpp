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

RobotModelRegistry<RobotModelPinocchio> RobotModelPinocchio::reg("pinocchio");

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
    base_frame =  robot_urdf->getRoot()->name;

    try{
        if(cfg.floating_base){
            pinocchio::urdf::buildModel(robot_urdf,pinocchio::JointModelFreeFlyer(), model);
        }
        else{
            pinocchio::urdf::buildModel(robot_urdf, model);
        }
    }
    catch(std::invalid_argument e){
        LOG_ERROR_S << "RobotModelPinocchio: Failed to load urdf model from " << cfg.file <<std::endl;
        return false;
    }
    data = std::make_shared<pinocchio::Data>(model);

    // Add floating base
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(has_floating_base){
        joint_names_floating_base = URDFTools::addFloatingBaseToURDF(robot_urdf);
        world_frame = robot_urdf->getRoot()->name;
    }

    joint_names = model.names;
    joint_names.erase(joint_names.begin()); // Erase global joint 'universe' which is added by Pinocchio
    if(has_floating_base)
        joint_names.erase(joint_names.begin()); // Erase 'floating_base' root joint
    actuated_joint_names = joint_names;
    joint_names = independent_joint_names = joint_names_floating_base + joint_names;

    // 2. Verify consistency of URDF and config

    // All contact point have to be a valid link in the robot URDF
    for(auto c : cfg.contact_points.names){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }

    // 3. Create data structures

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

    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(uint i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, jointIndex(actuated_joint_names[i])) = 1.0;

    contact_points = cfg.contact_points.names;
    active_contacts = cfg.contact_points;

    LOG_DEBUG("------------------- WBC RobotModelPinocchio -----------------");
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

    for(auto n : actuated_joint_names)
        joint_state[n] = joint_state_in[n];
    joint_state.time = joint_state_in.time;

    if(has_floating_base){
        if(!floating_base_state_in.hasValidPose() ||
           !floating_base_state_in.hasValidTwist() ||
           !floating_base_state_in.hasValidAcceleration()){
           LOG_ERROR("Invalid status of floating base given! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion)");
           throw std::runtime_error("Invalid floating base status");
        }
        if(floating_base_state_in.time.isNull()){
            LOG_ERROR("Floating base state does not have a valid timestamp. Or do we have 1970?");
            throw std::runtime_error("Invalid call to update()");
        }

        // Pinocchio expects the floating base twist/acceleration in local coordinates. However, we
        // want to give the linear part in world coordinates and the angular part in local coordinates
        floating_base_state = floating_base_state_in;
        base::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();

        base::Twist fb_twist = floating_base_state.twist;
        fb_twist.linear = fb_rot.transpose() * floating_base_state.twist.linear;

        base::Acceleration fb_acc = floating_base_state.acceleration;
        fb_acc.linear = fb_rot.transpose() * floating_base_state.acceleration.linear;

        base::Vector3d euler = floating_base_state.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        for(int i = 0; i < 3; i++){
            q[i]     = joint_state[joint_names_floating_base[i]].position       = floating_base_state.pose.position[i];
            joint_state[joint_names_floating_base[i+3]].position = euler(i);
            qd[i]    = joint_state[joint_names_floating_base[i]].speed          = fb_twist.linear[i];
            qd[i+3]  = joint_state[joint_names_floating_base[i+3]].speed        = fb_twist.angular[i];
            qdd[i]   = joint_state[joint_names_floating_base[i]].acceleration   = fb_acc.linear[i];
            qdd[i+3] = joint_state[joint_names_floating_base[i+3]].acceleration = fb_acc.angular[i];
        }
        q[3] = floating_base_state.pose.orientation.x();
        q[4] = floating_base_state.pose.orientation.y();
        q[5] = floating_base_state.pose.orientation.z();
        q[6] = floating_base_state.pose.orientation.w();

        // Subtract 2 due to universe & root_joint
        for(auto name : actuated_joint_names){
            if(!hasJoint(name)){
                LOG_ERROR_S << "Joint " << name << " is a non-fixed joint in the robot model, but it is not in the joint state vector."
                            << "You should either set the joint to 'fixed' in your URDF file or provide a valid joint state for it" << std::endl;
                throw std::runtime_error("Incomplete Joint State");
            }
            base::JointState state = joint_state[name];
            q[model.getJointId(name)-2+7]   = state.position;     // first 7 elements in q are floating base pose
            qd[model.getJointId(name)-2+6]  = state.speed;        // first 6 elements in q are floating base twist
            qdd[model.getJointId(name)-2+6] = state.acceleration; // first 6 elements in q are floating base acceleration
        }
        if(floating_base_state.time > joint_state.time)
            joint_state.time = floating_base_state.time;
    }
    else{
        for(auto name : actuated_joint_names){
            if(!hasJoint(name)){
                LOG_ERROR_S << "Joint " << name << " is a non-fixed joint in the robot model, but it is not in the joint state vector."
                            << "You should either set the joint to 'fixed' in your URDF file or provide a valid joint state for it" << std::endl;
                throw std::runtime_error("Incomplete Joint State");
            }
            base::JointState state = joint_state[name];
            q[model.getJointId(name)-1]   = state.position;
            qd[model.getJointId(name)-1]  = state.speed;
            qdd[model.getJointId(name)-1] = state.acceleration;
        }
    }

}

void RobotModelPinocchio::systemState(base::VectorXd &_q, base::VectorXd &_qd, base::VectorXd &_qdd){
    _q = q;
    _qd = qd;
    _qdd = qdd;
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

    std::string use_tip_frame = tip_frame;
    if(use_tip_frame == "world")
        use_tip_frame = "universe";

    uint idx = model.getFrameId(use_tip_frame);
    if(idx == model.frames.size()){
        LOG_ERROR_S<<"Requested Forward kinematics for tip frame "<<use_tip_frame<<" but this frame does not exist in Pinocchio"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }

    pinocchio::forwardKinematics(model,*data,q,qd,qdd);
    pinocchio::updateFramePlacement(model,*data,idx);

    rbs.time = joint_state.time;
    rbs.frame_id = root_frame;
    rbs.pose.position = data->oMf[idx].translation();
    rbs.pose.orientation = base::Quaterniond(data->oMf[idx].rotation());
    // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
    // but with axes aligned with the frame of the Universe. This a MIXED representation betwenn the LOCAL and the WORLD conventions.
    rbs.twist.linear = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.twist.angular = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    rbs.acceleration.linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.acceleration.angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();

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

    std::string use_tip_frame = tip_frame;
    if(use_tip_frame == "world")
        use_tip_frame = "universe";

    uint idx = model.getFrameId(use_tip_frame);
    if(idx == model.frames.size()){
        LOG_ERROR_S<<"Requested Forward kinematics for tip frame "<<use_tip_frame<<" but this frame does not exist in Pinocchio"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }

    std::string chain_id = chainID(root_frame, tip_frame);
    space_jac_map[chain_id].resize(6,model.nv);
    space_jac_map[chain_id].setZero();
    pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL_WORLD_ALIGNED, space_jac_map[chain_id]);

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

    std::string use_tip_frame = tip_frame;
    if(use_tip_frame == "world")
        use_tip_frame = "universe";

    uint idx = model.getFrameId(use_tip_frame);
    if(idx == model.frames.size()){
        LOG_ERROR_S<<"Requested Forward kinematics for tip frame "<<use_tip_frame<<" but this frame does not exist in Pinocchio"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }

    std::string chain_id = chainID(root_frame, tip_frame);
    body_jac_map[chain_id].resize(6,model.nv);
    body_jac_map[chain_id].setZero();
    pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL, body_jac_map[chain_id]);

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

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModelPinocchio: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != world_frame){
        LOG_ERROR_S<<"Requested space Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but the pinocchio robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    std::string use_tip_frame = tip_frame;
    if(use_tip_frame == "world")
        use_tip_frame = "universe";

    uint idx = model.getFrameId(use_tip_frame);
    if(idx == model.frames.size()){
        LOG_ERROR_S<<"Requested Forward kinematics for tip frame "<<use_tip_frame<<" but this frame does not exist in Pinocchio"<<std::endl;
        throw std::runtime_error("Invalid tip frame");
    }
    pinocchio::forwardKinematics(model,*data,q,qd,base::VectorXd::Zero(model.nv));
    spatial_acc_bias.linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    spatial_acc_bias.angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
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

    uint start_idx = 0;
    if(has_floating_base)
        start_idx = 6;

    for(uint i = 0; i < noOfJoints(); i++){
        const std::string &name = actuatedJointNames()[i];
        solver_output[name].effort = data->tau[i+start_idx];
    }
}

}
