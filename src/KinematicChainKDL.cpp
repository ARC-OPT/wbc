#include "KinematicChainKDL.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/Joints.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames_io.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain &chain) :
    chain(chain),
    jacobian(KDL::Jacobian(chain.getNrOfJoints())),
    jacobian_dot(KDL::Jacobian(chain.getNrOfJoints())){

    joint_state_kdl.q    = KDL::JntArray(chain.getNrOfJoints());
    joint_state_kdl.qdot = KDL::JntArray(chain.getNrOfJoints());

    for(size_t i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
}

const CartesianState &KinematicChainKDL::cartesianState(){
    cartesian_state.pose.position << pose_kdl.p(0), pose_kdl.p(1), pose_kdl.p(2);
    double x, y, z, w;
    pose_kdl.M.GetQuaternion(x, y, z, w);
    cartesian_state.pose.orientation = base::Quaterniond(w, x, y, z);
    cartesian_state.twist.linear  << twist_kdl.vel(0), twist_kdl.vel(1), twist_kdl.vel(2);
    cartesian_state.twist.angular << twist_kdl.rot(0), twist_kdl.rot(1), twist_kdl.rot(2);
    return cartesian_state;
}

void KinematicChainKDL::update(const base::samples::Joints &joint_state){

    //// update Joints
    for(size_t i = 0; i < joint_names.size(); i++)
        try{
            joint_state_kdl.q(i)    = joint_state.getElementByName(joint_names[i]).position;
            joint_state_kdl.qdot(i) = joint_state.getElementByName(joint_names[i]).speed;
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
    fk_solver_vel.JntToCart(joint_state_kdl, frame_vel);
    twist_kdl = frame_vel.deriv();
    pose_kdl = frame_vel.value();

    //// Compute Jacobians
    jac_solver.JntToJac(joint_state_kdl.q, jacobian);

    jac_dot_solver.setRepresentation(0); // 0 - Hybrid represenation -> ref frame is root, ref point is tip
    jac_dot_solver.JntToJacDot(joint_state_kdl, jacobian_dot);
}


} // namespace wbc
