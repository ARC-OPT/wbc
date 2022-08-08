#include "KinematicChainKDL.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/Joints.hpp>
#include <kdl/chaindynparam.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain &_chain, const std::string &_root_frame, const std::string &_tip_frame) :
    chain(_chain),
    root_frame(_root_frame),
    tip_frame(_tip_frame),
    space_jacobian(KDL::Jacobian(chain.getNrOfJoints())),
    body_jacobian(KDL::Jacobian(chain.getNrOfJoints())),
    jacobian_dot(KDL::Jacobian(chain.getNrOfJoints())){

    jnt_array_vel.q    = KDL::JntArray(chain.getNrOfJoints());
    jnt_array_vel.qdot = KDL::JntArray(chain.getNrOfJoints());

    jnt_array_acc.q       = KDL::JntArray(chain.getNrOfJoints());
    jnt_array_acc.qdot    = KDL::JntArray(chain.getNrOfJoints());
    jnt_array_acc.qdotdot = KDL::JntArray(chain.getNrOfJoints());
    for(size_t i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }

    cartesian_state.frame_id = root_frame;

    has_acceleration = space_jacobian_is_up_to_date = body_jacobian_is_up_to_date = jac_dot_is_up_to_date = false;

    jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(chain);
    jac_dot_solver = std::make_shared<KDL::ChainJntToJacDotSolver>(chain);
    fk_solver_vel = std::make_shared<KDL::ChainFkSolverVel_recursive>(chain);
}

const base::samples::RigidBodyStateSE3 &KinematicChainKDL::rigidBodyState(){
    cartesian_state.pose.position << pose_kdl.p(0), pose_kdl.p(1), pose_kdl.p(2);
    double x, y, z, w;
    pose_kdl.M.GetQuaternion(x, y, z, w);
    cartesian_state.pose.orientation = base::Quaterniond(w, x, y, z);
    cartesian_state.twist.linear  << twist_kdl.vel(0), twist_kdl.vel(1), twist_kdl.vel(2);
    cartesian_state.twist.angular << twist_kdl.rot(0), twist_kdl.rot(1), twist_kdl.rot(2);
    cartesian_state.acceleration.linear = acc.segment(0,3);
    cartesian_state.acceleration.angular = acc.segment(3,3);
    return cartesian_state;
}

void KinematicChainKDL::update(const KDL::JntArray& q, const KDL::JntArray& qd, const KDL::JntArray& qdd, std::map<std::string,int> joint_idx_map){

    //// update Joints
    for(size_t i = 0; i < joint_names.size(); i++){
        const std::string& name = joint_names[i];
        if(joint_idx_map.count(name) == 0){
            LOG_ERROR("Kinematic Chain %s to %s contains joint %s, but this joint is not in joint state vector",
                      chain.getSegment(0).getName().c_str(), chain.getSegment(chain.getNrOfSegments()-1).getName().c_str(), joint_names[i].c_str());
            throw std::invalid_argument("Invalid joint state");
        }

        uint idx = joint_idx_map[name];

        jnt_array_vel.q(i)       = jnt_array_acc.q(i)    = q(idx);
        jnt_array_vel.qdot(i)    = jnt_array_acc.qdot(i) = qd(idx);
        jnt_array_acc.qdotdot(i) = qdd(idx);
    }

    space_jacobian_is_up_to_date = body_jacobian_is_up_to_date = jac_dot_is_up_to_date = fk_is_up_to_date = false;
}

void KinematicChainKDL::calculateForwardKinematics(){
    fk_solver_vel->JntToCart(jnt_array_vel, frame_vel);

    twist_kdl = frame_vel.deriv();
    pose_kdl = frame_vel.value();

    // Update jacobians if necessary
    if(!space_jacobian_is_up_to_date)
        calculateSpaceJacobian();
    if(!jac_dot_is_up_to_date)
        calculateJacobianDot();

    acc = jacobian_dot.data*jnt_array_vel.qdot.data + space_jacobian.data*jnt_array_acc.qdotdot.data;

    fk_is_up_to_date = true;
}

void KinematicChainKDL::calculateSpaceJacobian(){
    if(jac_solver->JntToJac(jnt_array_vel.q, space_jacobian))
        throw std::runtime_error("Failed to compute Jacobian for chain " + root_frame + " -> " + tip_frame);
    space_jacobian_is_up_to_date = true;
}

void KinematicChainKDL::calculateBodyJacobian(){
    if(!fk_is_up_to_date)
        calculateForwardKinematics();
    if(!space_jacobian_is_up_to_date)
        calculateSpaceJacobian();
    body_jacobian = space_jacobian;
    body_jacobian.changeBase(pose_kdl.M.Inverse());
    body_jacobian_is_up_to_date = true;
}

void KinematicChainKDL::calculateJacobianDot(){
    jac_dot_solver->setRepresentation(KDL::ChainJntToJacDotSolver::HYBRID); // 0 - Hybrid represenation -> ref frame is root, ref point is tip
    if(jac_dot_solver->JntToJacDot(jnt_array_vel, jacobian_dot))
        throw std::runtime_error("Failed to compute JacobianDot for chain " + root_frame + " -> " + tip_frame);
    jac_dot_is_up_to_date = true;
}

} // namespace wbc
