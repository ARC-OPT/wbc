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

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits, joint_names);

    selection_matrix.resize(na(),nj());
    selection_matrix.setZero();
    for(uint i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, nfb() + i) = 1.0;

    contacts = cfg.contact_points;

    configured = true;
    compute_com = compute_com_jac = compute_bias_forces = compute_inertia_mat = false;


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
    updated = true;

    pinocchio::forwardKinematics(model,*data,q,qd,qdd);

    if(compute_inertia_mat)
        jointSpaceInertiaMatrix(true);
    if(compute_bias_forces)
        biasForces(true);
    for(auto it : pose_map)
        pose(it.first,true);
    for(auto it : twist_map)
        twist(it.first,true);
    for(auto it : acc_map)
        acceleration(it.first,true);
    for(auto it : space_jac_map)
        spaceJacobian(it.first,true);
    for(auto it : body_jac_map)
        bodyJacobian(it.first,true);
    if(compute_com)
        centerOfMass(true);
    if(compute_com_jac)
        comJacobian(true);
    pinocchio::forwardKinematics(model,*data,q,qd,Eigen::VectorXd::Zero(model.nv));
    for(auto it : spatial_acc_bias_map)
        spatialAccelerationBias(it.first,true);

}

const types::Pose &RobotModelPinocchio::pose(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || pose_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        pinocchio::updateFramePlacement(model,*data,idx);

        pose_map[frame_id].position = data->oMf[idx].translation();
        pose_map[frame_id].orientation = Eigen::Quaterniond(data->oMf[idx].rotation());
    }

    return pose_map[frame_id];
}

const types::Twist &RobotModelPinocchio::twist(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || twist_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        pinocchio::updateFramePlacement(model,*data,idx);

        // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
        // but with axes aligned with the world frame. This a MIXED representation betwenn the LOCAL and the WORLD.
        twist_map[frame_id].linear = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        twist_map[frame_id].angular = pinocchio::getFrameVelocity(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    }

    return twist_map[frame_id];
}

const types::SpatialAcceleration &RobotModelPinocchio::acceleration(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || acc_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        pinocchio::updateFramePlacement(model,*data,idx);

        // The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.)
        // but with axes aligned with the world frame. This a MIXED representation betwenn the LOCAL and the WORLD.
        acc_map[frame_id].linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        acc_map[frame_id].angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    }

    return acc_map[frame_id];
}

const Eigen::MatrixXd &RobotModelPinocchio::spaceJacobian(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || space_jac_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        space_jac_map[frame_id].resize(6,model.nv);
        space_jac_map[frame_id].setZero();
        pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL_WORLD_ALIGNED, space_jac_map[frame_id]);
    }

    return space_jac_map[frame_id];
}

const Eigen::MatrixXd &RobotModelPinocchio::bodyJacobian(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || body_jac_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        body_jac_map[frame_id].resize(6,model.nv);
        body_jac_map[frame_id].setZero();
        pinocchio::computeFrameJacobian(model, *data, q, idx, pinocchio::LOCAL, body_jac_map[frame_id]);
    }

    return body_jac_map[frame_id];
}

const Eigen::MatrixXd &RobotModelPinocchio::comJacobian(const bool recompute){

    assert(updated);
    compute_com_jac = true;

    if(recompute || com_jac.rows() == 0){

        pinocchio::jacobianCenterOfMass(model, *data, q);
        com_jac.resize(3,model.nv);
        com_jac = data->Jcom;
    }
    return com_jac;
}

const types::SpatialAcceleration &RobotModelPinocchio::spatialAccelerationBias(const std::string &frame_id, const bool recompute){

    assert(updated);
    assert(frame_id != "world");

    if(recompute || spatial_acc_bias_map.count(frame_id) == 0){

        uint idx = model.getFrameId(frame_id);
        assert(idx != model.frames.size());

        spatial_acc_bias_map[frame_id].linear = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        spatial_acc_bias_map[frame_id].angular = pinocchio::getFrameClassicalAcceleration(model, *data, idx, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    }

    return spatial_acc_bias_map[frame_id];
}

const Eigen::MatrixXd &RobotModelPinocchio::jointSpaceInertiaMatrix(const bool recompute){

    assert(updated);    
    compute_inertia_mat = true;

    if(recompute || joint_space_inertia_mat.rows() == 0){

        pinocchio::crba(model, *data, q);
        joint_space_inertia_mat = data->M;
        // copy upper right triangular part to lower left triangular part (they are symmetric), as pinocchio only computes the former
        joint_space_inertia_mat.triangularView<Eigen::StrictlyLower>() = joint_space_inertia_mat.transpose().triangularView<Eigen::StrictlyLower>();
    }
    return joint_space_inertia_mat;
}

const Eigen::VectorXd &RobotModelPinocchio::biasForces(const bool recompute){

    assert(updated);
    compute_bias_forces = true;

    if(recompute || bias_forces.rows() == 0){

        pinocchio::nonLinearEffects(model, *data, q, qd);
        bias_forces = data->nle;
    }
    return bias_forces;
}

const types::RigidBodyState& RobotModelPinocchio::centerOfMass(const bool recompute){

    assert(updated);
    compute_com = true;

    if(recompute){
        pinocchio::centerOfMass(model, *data, q, qd, qdd);
        com_rbs.pose.position       = data->com[0];
        com_rbs.twist.linear        = data->vcom[0];
        com_rbs.acceleration.linear = data->acom[0];
        com_rbs.pose.orientation.setIdentity();
        com_rbs.twist.angular.setZero();
        com_rbs.acceleration.angular.setZero();
    }
    return com_rbs;
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
