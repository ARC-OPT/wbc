#include "RobotModelPinocchio.hpp"
#include "../../tools/URDFTools.hpp"
#include "../../tools/Logger.hpp"
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
    robot_urdf = loadRobotURDF(cfg.file_or_string);
    if(!robot_urdf){
        log(logERROR)<<"Unable to parse urdf model";
        return false;
    }
    base_frame =  robot_urdf->getRoot()->name;
    if(!URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist))
        return false;

    try{
        if(cfg.floating_base){
            pinocchio::urdf::buildModel(robot_urdf,pinocchio::JointModelFreeFlyer(), model);
        }
        else{
            pinocchio::urdf::buildModel(robot_urdf, model);
        }
    }
    catch(std::invalid_argument e){
        log(logERROR) << "RobotModelPinocchio: Failed to load urdf model";
        return false;
    }
    data = std::make_shared<pinocchio::Data>(model);

    // Add floating base if available, world frame will be root of floating base instead of robot base frame
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(has_floating_base)
        world_frame = "world";

    joint_names = model.names;
    joint_names.erase(joint_names.begin()); // Erase global joint 'universe' which is added by Pinocchio
    if(has_floating_base)
        joint_names.erase(joint_names.begin()); // Erase 'floating_base' root joint
    actuated_joint_names = joint_names;
    independent_joint_names = joint_names;

    // 2. Create data structures

    // Joint order in q,qd,qdd:
    //
    // Floating base:
    // q:   x,y,z,qx,qy,qz,qw,joints_q,         Size: 7 + joint_names.size()
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
    zero_jnt.setZero(model.nv);
    zero_acc.setZero();

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits, joint_names);

    selection_matrix.resize(na(),nj());
    selection_matrix.setZero();
    for(uint i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, nfb() + i) = 1.0;

    contacts = cfg.contact_points;

    configured = true;

    // 3. Verify consistency of URDF and config

    // All contact points have to be a valid link in the robot URDF
    for(auto c : contacts){
        if(!hasLink(c.frame_id)){
            log(logERROR)<<"Contact point "<<c.frame_id<<" is not a valid link in the robot model";
            return false;
        }
    }

    return true;
}

void RobotModelPinocchio::update(const Eigen::VectorXd& joint_positions,
                                 const Eigen::VectorXd& joint_velocities,
                                 const Eigen::VectorXd& joint_accelerations,
                                 const types::Pose& fb_pose,
                                 const types::Twist& fb_twist,
                                 const types::SpatialAcceleration& fb_acc){

    assert(configured);
    assert(joint_positions.size() == na());
    assert(joint_velocities.size() == na());
    assert(joint_accelerations.size() == na());

    resetData();

    joint_state.position = joint_positions;
    joint_state.velocity = joint_velocities;
    joint_state.acceleration = joint_accelerations;

    if(has_floating_base){      
        // Pinocchio expects the floating base twist/acceleration in local coordinates. However, we
        // want to give the linear part in world coordinates and the angular part in local coordinates, aligned wrt. world
        floating_base_state.pose = fb_pose;
        floating_base_state.twist = fb_twist;
        floating_base_state.acceleration = fb_acc;
        Eigen::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();

        types::Twist fb_twist_tmp = floating_base_state.twist;
        fb_twist_tmp.linear = fb_rot.transpose() * floating_base_state.twist.linear;
        fb_twist_tmp.angular = fb_rot.transpose() * floating_base_state.twist.angular;

        types::SpatialAcceleration fb_acc_tmp = floating_base_state.acceleration;
        fb_acc_tmp.linear = fb_rot.transpose() * floating_base_state.acceleration.linear;
        fb_acc_tmp.angular = fb_rot.transpose() * floating_base_state.acceleration.angular;

        for(int i = 0; i < 3; i++){
            q[i]     = floating_base_state.pose.position[i];
            qd[i]    = fb_twist_tmp.linear[i];
            qd[i+3]  = fb_twist_tmp.angular[i];
            qdd[i]   = fb_acc_tmp.linear[i];
            qdd[i+3] = fb_acc_tmp.angular[i];
        }
        q[3] = floating_base_state.pose.orientation.x();
        q[4] = floating_base_state.pose.orientation.y();
        q[5] = floating_base_state.pose.orientation.z();
        q[6] = floating_base_state.pose.orientation.w();

        // Subtract 2 due to universe & root_joint
        for(size_t i = 0; i < actuatedJointNames().size(); i++){
            q[model.getJointId(actuated_joint_names[i])-2+7]   = joint_state.position[i];     // first 7 elements in q are floating base pose
            qd[model.getJointId(actuated_joint_names[i])-2+6]  = joint_state.velocity[i];        // first 6 elements in q are floating base twist
            qdd[model.getJointId(actuated_joint_names[i])-2+6] = joint_state.acceleration[i]; // first 6 elements in q are floating base acceleration
        }
    }
    else{
        for(size_t i = 0; i < actuatedJointNames().size(); i++){
            q[model.getJointId(actuated_joint_names[i])-1]   = joint_state.position[i];
            qd[model.getJointId(actuated_joint_names[i])-1]  = joint_state.velocity[i];
            qdd[model.getJointId(actuated_joint_names[i])-1] = joint_state.acceleration[i];
        }
    }

    updateData();
    updated = true;
}


void RobotModelPinocchio::updateFK(Eigen::VectorXd &_q,Eigen::VectorXd &_qd,Eigen::VectorXd &_qdd){
    pinocchio::forwardKinematics(model,*data,_q,_qd,_qdd);
    fk_needs_recompute = false;
    fk_with_zero_acc_needs_recompute = true;
}

void RobotModelPinocchio::updateFK(Eigen::VectorXd &_q,Eigen::VectorXd &_qd){
    pinocchio::forwardKinematics(model,*data,_q,_qd,zero_jnt);
    fk_needs_recompute = true;
    fk_with_zero_acc_needs_recompute = false;
}

const types::Pose &RobotModelPinocchio::pose(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(pose_map.count(frame_id) == 0)
        pose_map[frame_id] = Pose();

    if(pose_map[frame_id].needs_recompute){
        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        if(fk_needs_recompute)
            updateFK(q,qd,qdd);

        pinocchio::updateFramePlacement(model,*data,idx);

        pose_map[frame_id].data.position = data->oMf[idx].translation();
        pose_map[frame_id].data.orientation = Eigen::Quaterniond(data->oMf[idx].rotation());
        pose_map[frame_id].needs_recompute = false;
    }

    return pose_map[frame_id].data;
}

const types::Twist &RobotModelPinocchio::twist(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(twist_map.count(frame_id) == 0)
        twist_map[frame_id] = Twist();

    if(twist_map[frame_id].needs_recompute){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        if(fk_needs_recompute)
            updateFK(q,qd,qdd);

        pinocchio::updateFramePlacement(model,*data,idx);

        // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
        // but with axes aligned with the world frame. This a MIXED representation betwenn the LOCAL and the WORLD.
        twist_map[frame_id].data.linear = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        twist_map[frame_id].data.angular = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
        twist_map[frame_id].needs_recompute = false;
    }

    return twist_map[frame_id].data;
}

const types::SpatialAcceleration &RobotModelPinocchio::acceleration(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(acc_map.count(frame_id) == 0)
        acc_map[frame_id] = SpatialAcceleration();

    if(acc_map[frame_id].needs_recompute){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        if(fk_needs_recompute)
            updateFK(q,qd,qdd);

        pinocchio::updateFramePlacement(model,*data,idx);

        // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
        // but with axes aligned with the world frame. This a MIXED representation betwenn the LOCAL and the WORLD.
        acc_map[frame_id].data.linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        acc_map[frame_id].data.angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
        acc_map[frame_id].needs_recompute = false;
    }

    return acc_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelPinocchio::spaceJacobian(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(space_jac_map.count(frame_id) == 0)
        space_jac_map[frame_id] = Matrix();

    if(space_jac_map[frame_id].needs_recompute){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        space_jac_map[frame_id].data.resize(6,model.nv);
        space_jac_map[frame_id].data.setZero();
        pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL_WORLD_ALIGNED, space_jac_map[frame_id].data);
        space_jac_map[frame_id].needs_recompute = false;
    }

    return space_jac_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelPinocchio::bodyJacobian(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(body_jac_map.count(frame_id) == 0)
        body_jac_map[frame_id] = Matrix();

    if(body_jac_map[frame_id].needs_recompute){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        body_jac_map[frame_id].data.resize(6,model.nv);
        body_jac_map[frame_id].data.setZero();
        pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL, body_jac_map[frame_id].data);
        body_jac_map[frame_id].needs_recompute = false;
    }

    return body_jac_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelPinocchio::comJacobian(){

    assert(updated);

    if(com_jac.empty())
        com_jac.push_back(Matrix());

    if(com_jac[0].needs_recompute){

        pinocchio::jacobianCenterOfMass(model, *data, q);
        com_jac[0].data.resize(3,model.nv);
        com_jac[0].data = data->Jcom;
        com_jac[0].needs_recompute = false;
    }

    return com_jac[0].data;
}

const types::SpatialAcceleration &RobotModelPinocchio::spatialAccelerationBias(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    if(spatial_acc_bias_map.count(frame_id) == 0)
        spatial_acc_bias_map[frame_id] = SpatialAcceleration();

    if(spatial_acc_bias_map[frame_id].needs_recompute){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        if(fk_with_zero_acc_needs_recompute)
            updateFK(q,qd);

        pinocchio::updateFramePlacement(model,*data,idx);

        spatial_acc_bias_map[frame_id].data.linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        spatial_acc_bias_map[frame_id].data.angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
        spatial_acc_bias_map[frame_id].needs_recompute = false;
    }

    return spatial_acc_bias_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelPinocchio::jointSpaceInertiaMatrix(){

    assert(updated);

    if(joint_space_inertia_mat.empty())
        joint_space_inertia_mat.push_back(Matrix());

    if(joint_space_inertia_mat[0].needs_recompute){
        pinocchio::crba(model, *data, q);
        joint_space_inertia_mat[0].data = data->M;

        // copy upper right triangular part to lower left triangular part (they are symmetric), as pinocchio only computes the former
        joint_space_inertia_mat[0].data.triangularView<Eigen::StrictlyLower>() = joint_space_inertia_mat[0].data.transpose().triangularView<Eigen::StrictlyLower>();
        joint_space_inertia_mat[0].needs_recompute = false;
    }

    return joint_space_inertia_mat[0].data;
}

const Eigen::VectorXd &RobotModelPinocchio::biasForces(){

    assert(updated);

    if(bias_forces.empty())
        bias_forces.push_back(Vector());

    if(bias_forces[0].needs_recompute){
        pinocchio::nonLinearEffects(model, *data, q, qd);
        bias_forces[0].data = data->nle;
        bias_forces[0].needs_recompute = false;
    }

    return bias_forces[0].data;
}

const types::RigidBodyState& RobotModelPinocchio::centerOfMass(){

    assert(updated);

    if(com_rbs.empty())
        com_rbs.push_back(RigidBodyState());

    if(com_rbs[0].needs_recompute){

        pinocchio::centerOfMass(model, *data, q, qd, qdd);
        com_rbs[0].data.pose.position       = data->com[0];
        com_rbs[0].data.twist.linear        = data->vcom[0];
        com_rbs[0].data.acceleration.linear = data->acom[0];
        com_rbs[0].data.pose.orientation.setIdentity();
        com_rbs[0].data.twist.angular.setZero();
        com_rbs[0].data.acceleration.angular.setZero();
        com_rbs[0].needs_recompute = false;
    }

    return com_rbs[0].data;
}

const Eigen::VectorXd& RobotModelPinocchio::inverseDynamics(const Eigen::VectorXd& qdd_ref, const std::vector<types::Wrench>& f_ext){

    assert(updated);

    if(qdd_ref.size() == 0)
        tau = pinocchio::rnea(model, *data, q, qd, qdd);
    else
        tau = pinocchio::rnea(model, *data, q, qd, qdd_ref);

    if(f_ext.size() != 0){
        assert(f_ext.size() == contacts.size());
        for(size_t i = 0; i < contacts.size(); i++)
            tau += spaceJacobian(contacts[i].frame_id)*f_ext[i].vector6d();
    }

    return tau;
}

}
