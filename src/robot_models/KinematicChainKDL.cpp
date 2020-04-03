#include "KinematicChainKDL.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/Joints.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames_io.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain &chain, const std::string &root_frame, const std::string &tip_frame) :
    chain(chain),
    jacobian(KDL::Jacobian(chain.getNrOfJoints())),
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

void KinematicChainKDL::update(const base::samples::Joints &joint_state){

    //// update Joints
    cartesian_state.time = joint_state.time;
    bool has_acceleration = true;
    for(size_t i = 0; i < joint_names.size(); i++)
        try{
            jnt_array_vel.q(i)       = jnt_array_acc.q(i)    = joint_state.getElementByName(joint_names[i]).position;
            jnt_array_vel.qdot(i)    = jnt_array_acc.qdot(i) = joint_state.getElementByName(joint_names[i]).speed;
            jnt_array_acc.qdotdot(i) = joint_state.getElementByName(joint_names[i]).acceleration;

            has_acceleration = has_acceleration && joint_state.getElementByName(joint_names[i]).hasAcceleration();
        }
        catch(std::exception e){
            LOG_ERROR("Kinematic Chain %s to %s contains joint %s, but this joint is not in joint state vector",
                      chain.getSegment(0).getName().c_str(), chain.getSegment(chain.getNrOfSegments()-1).getName().c_str(), joint_names[i].c_str());
            throw std::invalid_argument("Invalid joint state");
        }

    KDL::ChainFkSolverVel_recursive fk_solver_vel(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);
    KDL::ChainJntToJacDotSolver jac_dot_solver(chain);

    //// compute forward kinematics
    fk_solver_vel.JntToCart(jnt_array_vel, frame_vel);
    twist_kdl = frame_vel.deriv();
    pose_kdl = frame_vel.value();

    //// Compute Jacobian
    jac_solver.JntToJac(jnt_array_vel.q, jacobian);

    //// Compute Jacobian_dot
    jac_dot_solver.setRepresentation(0); // 0 - Hybrid represenation -> ref frame is root, ref point is tip
    jac_dot_solver.JntToJacDot(jnt_array_vel, jacobian_dot);

    //// Compute frame acceleration
    if(has_acceleration)
        acc = jacobian_dot.data*jnt_array_vel.qdot.data + jacobian.data*jnt_array_acc.qdotdot.data;
}


} // namespace wbc
