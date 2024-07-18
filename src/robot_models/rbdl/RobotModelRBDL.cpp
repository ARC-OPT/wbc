#include "RobotModelRBDL.hpp"
#include "tools/URDFTools.hpp"
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "../tools/Logger.hpp"

using namespace RigidBodyDynamics;

namespace wbc {

RobotModelRegistry<RobotModelRBDL> RobotModelRBDL::reg("rbdl");

RobotModelRBDL::RobotModelRBDL(){

}

RobotModelRBDL::~RobotModelRBDL(){

}

void RobotModelRBDL::clear(){
    rbdl_model.reset();
    rbdl_model = std::make_shared<Model>();
}

bool RobotModelRBDL::configure(const RobotModelConfig& cfg){

    clear();

    // 1. Load Robot Model

    robot_model_config = cfg;
    robot_urdf = loadRobotURDF(cfg.file_or_string);
    if(!robot_urdf){
        log(logERROR)<<"Unable to parse urdf model";
        return false;
    }
    base_frame = robot_urdf->getRoot()->name;
    URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist);
    joint_names = URDFTools::jointNamesFromURDF(robot_urdf);

    // Temporary workaround: RBDL does not support rotations of the link invertias. Set them to zero.
    // Note: This will lead to inconsistent behavior in acceleration-Eigend WBC
    for(auto &l : robot_urdf->links_){
        if(l.second->inertial)
            l.second->inertial->origin.rotation.setFromRPY(0,0,0);
    }
    TiXmlDocument *doc = urdf::exportURDF(robot_urdf);
    std::string robot_urdf_file = "/tmp/robot.urdf";
    doc->SaveFile(robot_urdf_file);

    if(!Addons::URDFReadFromFile(robot_urdf_file.c_str(), rbdl_model.get(), cfg.floating_base)){
        log(logERROR) << "Unable to parse urdf from file " << robot_urdf_file;
        return false;
    }

    // Add floating Eigen to robot_urdf. We will not load this URDF model in RBDL, since RBDL adds its own floating Eigen.
    // However, we need the robot_urdf for some internal functionalities.
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(cfg.floating_base)
        world_frame = "world";

    actuated_joint_names = joint_names;
    independent_joint_names = joint_names;

    // Read Joint Limits
    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits, joint_names);

    // 2. Create data structures

    joint_state.resize(joint_names.size());

    // Fixed Eigen: If the robot has N dof, q_size = qd_size = N
    // Floating Eigen: If the robot has N dof, q_size = N+7, qd_size = N+6
    q.resize(rbdl_model->q_size);
    qd.resize(rbdl_model->qdot_size);
    qdd.resize(rbdl_model->qdot_size);
    tau.resize(rbdl_model->qdot_size);

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

void RobotModelRBDL::update(const Eigen::VectorXd& joint_positions,
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

    uint start_idx = 0;
    if(has_floating_base){
        floating_base_state.pose = fb_pose;
        floating_base_state.twist = fb_twist;
        floating_base_state.acceleration = fb_acc;
        start_idx = 6;

        // Transformation from fb body linear acceleration to fb joint linear acceleration:
        // since RBDL treats the floating Eigen as XYZ translation followed by spherical
        // aj is S*qdd(i)
        // j is parent of i
        // a(i) = Xa(j) + aj + cross(v(i), vj)
        // For pinocchio we give directly a(fb), for RBDL we give aj instead (for the first two joints)
        // so we have to remove the cross contribution cross(v(i), vj) from it
        Eigen::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();
        types::Twist fb_twist_tmp = floating_base_state.twist;
        types::SpatialAcceleration fb_acc_tmp = floating_base_state.acceleration;

        Eigen::VectorXd spherical_j_vel(6);
        spherical_j_vel << fb_twist_tmp.angular, Eigen::Vector3d::Zero();
        Eigen::VectorXd spherical_b_vel(6);
        spherical_b_vel << fb_twist_tmp.angular, fb_rot.transpose() * fb_twist_tmp.linear;

        Eigen::VectorXd fb_spherical_cross = Math::crossm(spherical_b_vel, spherical_j_vel);
        // remove cross contribution from linear acc s(in world coordinates as RBDL want)
        fb_acc_tmp.linear = fb_acc_tmp.linear - fb_rot * fb_spherical_cross.tail<3>();

        int floating_body_id = rbdl_model->GetBodyId(base_frame.c_str());
        rbdl_model->SetQuaternion(floating_body_id, Math::Quaternion(floating_base_state.pose.orientation.coeffs()), q);
        q.segment(0,3) = floating_base_state.pose.position;
        qd.segment(0,3) =  fb_twist_tmp.linear;
        qdd.segment(0,3) = fb_acc_tmp.linear;
        qd.segment(3,3) = fb_twist_tmp.angular;
        qdd.segment(3,3) = fb_acc_tmp.angular;
    }

    uint na = this->na();
    q.segment(start_idx,na) = joint_state.position;
    qd.segment(start_idx,na) = joint_state.velocity;
    qdd.segment(start_idx,na) = joint_state.acceleration;

    UpdateKinematics(*rbdl_model, q, qd, qdd); // update bodies kinematics once

    updated = true;
}

const types::Pose &RobotModelRBDL::pose(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    uint body_id = rbdl_model->GetBodyId(frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    rbs.pose.position = CalcBodyToBaseCoordinates(*rbdl_model, q,body_id, Eigen::Vector3d(0,0,0), false);
    rbs.pose.orientation = Eigen::Quaterniond(CalcBodyWorldOrientation(*rbdl_model, q, body_id, false).inverse());
    return rbs.pose;

}

const types::Twist &RobotModelRBDL::twist(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    uint body_id = rbdl_model->GetBodyId(frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    Math::SpatialVector twist_rbdl = CalcPointVelocity6D(*rbdl_model, q, qd, body_id, Eigen::Vector3d(0,0,0), false);
    rbs.twist.linear = twist_rbdl.segment(3,3);
    rbs.twist.angular = twist_rbdl.segment(0,3);
    return rbs.twist;
}

const types::SpatialAcceleration &RobotModelRBDL::acceleration(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    uint body_id = rbdl_model->GetBodyId(frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    Math::SpatialVector acc_rbdl = CalcPointAcceleration6D(*rbdl_model, q, qd, qdd, body_id, Eigen::Vector3d(0,0,0), false);
    rbs.acceleration.linear = acc_rbdl.segment(3,3);
    rbs.acceleration.angular = acc_rbdl.segment(0,3);
    return rbs.acceleration;
}

const Eigen::MatrixXd &RobotModelRBDL::spaceJacobian(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    uint body_id = rbdl_model->GetBodyId(frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    uint nj = rbdl_model->dof_count;
    space_jac.resize(6,nj);
    space_jac.setZero();

    Eigen::Vector3d point_position;
    point_position.setZero();
    J.setZero(6, nj);
    CalcPointJacobian6D(*rbdl_model, q, body_id, point_position, J, false);

    space_jac.block(0,0,3,nj) = J.block(3,0,3,nj);
    space_jac.block(3,0,3,nj) = J.block(0,0,3,nj);

    return space_jac;
}

const Eigen::MatrixXd &RobotModelRBDL::bodyJacobian(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    std::string use_frame_id = frame_id;
    if(use_frame_id == "world")
        use_frame_id = "ROOT";

    uint body_id = rbdl_model->GetBodyId(use_frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    uint nj = rbdl_model->dof_count;
    body_jac.resize(6,nj);
    body_jac.setZero();

    J.setZero(6, nj);
    CalcBodySpatialJacobian(*rbdl_model, q, body_id, J, false);

    body_jac.block(0,0,3,nj) = J.block(3,0,3,nj);
    body_jac.block(3,0,3,nj) = J.block(0,0,3,nj);

    return body_jac;
}

const Eigen::MatrixXd &RobotModelRBDL::comJacobian(){

    assert(updated);

    com_jac.setZero(3, rbdl_model->dof_count);
    double total_mass = 0.0;

    // iterate over the moving bodies except Eigen link (numbered 0 in the graph)
    for (unsigned int i = 1; i < rbdl_model->mBodies.size(); i++){
        const Body& body = rbdl_model->mBodies.at(i);
        Math::MatrixNd com_jac_body_i;
        com_jac_body_i.setZero(3, rbdl_model->dof_count);
        CalcPointJacobian(*rbdl_model, q, i, body.mCenterOfMass, com_jac_body_i, false);

        com_jac = com_jac + body.mMass * com_jac_body_i;
        total_mass = total_mass + body.mMass;
    }

    com_jac = (1/total_mass) * com_jac;
    return com_jac;
}

const types::SpatialAcceleration &RobotModelRBDL::spatialAccelerationBias(const std::string &frame_id){

    assert(updated);
    assert(frame_id != "world");

    uint body_id = rbdl_model->GetBodyId(frame_id.c_str());
    assert(body_id != std::numeric_limits<unsigned int>::max());

    Eigen::Vector3d point_position;
    point_position.setZero();
    Math::SpatialVector spatial_acceleration = CalcPointAcceleration6D(*rbdl_model, q, qd, Math::VectorNd::Zero(rbdl_model->dof_count), body_id, point_position, true);
    UpdateKinematics(*rbdl_model, q, qd, qdd);
    spatial_acc_bias.linear = spatial_acceleration.segment(3,3);
    spatial_acc_bias.angular = spatial_acceleration.segment(0,3);
    return spatial_acc_bias;
}

const Eigen::MatrixXd &RobotModelRBDL::jointSpaceInertiaMatrix(){

    assert(updated);

    H_q.setZero(rbdl_model->dof_count, rbdl_model->dof_count);
    CompositeRigidBodyAlgorithm(*rbdl_model, q, H_q, false);
    joint_space_inertia_mat = H_q;
    return joint_space_inertia_mat;
}

const Eigen::VectorXd &RobotModelRBDL::biasForces(){

    assert(updated);

    tau.resize(rbdl_model->dof_count);
    InverseDynamics(*rbdl_model, q, qd, Math::VectorNd::Zero(rbdl_model->dof_count), tau);
    bias_forces = tau;
    return bias_forces;
}

const types::RigidBodyState& RobotModelRBDL::centerOfMass(){

    assert(updated);

    double mass;
    Math::Vector3d com_pos, com_vel, com_acc;
    Utils::CalcCenterOfMass(*rbdl_model, q, qd, &qdd, mass, com_pos, &com_vel, &com_acc, nullptr, nullptr, false);

    com_rbs.pose.position = com_pos;
    com_rbs.pose.orientation.setIdentity();
    com_rbs.twist.linear = com_vel;
    com_rbs.twist.angular.setZero();
    com_rbs.acceleration.linear = com_acc; // TODO: double check CoM acceleration
    com_rbs.acceleration.angular.setZero();
    return com_rbs;
}

const Eigen::VectorXd& RobotModelRBDL::inverseDynamics(const Eigen::VectorXd& qdd_ref, const std::vector<types::Wrench>& f_ext){

    assert(updated);

    if(qdd_ref.size() == 0)
        InverseDynamics(*rbdl_model, q, qd, qdd, tau);
    else
        InverseDynamics(*rbdl_model, q, qd, qdd_ref, tau);

    if(f_ext.size() != 0){
        assert(f_ext.size() == contacts.size());
        for(size_t i = 0; i < contacts.size(); i++)
            tau += spaceJacobian(contacts[i].frame_id)*f_ext[i].vector6d();
    }

    return tau;
}

}
