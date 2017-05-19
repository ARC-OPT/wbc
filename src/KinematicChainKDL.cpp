#include "KinematicChainKDL.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl_conversions/KDLConversions.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(KDL::Chain chain) :
    chain(chain),
    joint_positions(KDL::JntArray(chain.getNrOfJoints())),
    jacobian(KDL::Jacobian(chain.getNrOfJoints())){

    for(size_t i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
}

KinematicChainKDL::~KinematicChainKDL(){

}

void KinematicChainKDL::update(const base::samples::Joints &joint_state, const std::vector<base::samples::RigidBodyState> &poses){

    //// update Joints
    last_update = joint_state.time;
    for(size_t i = 0; i < joint_names.size(); i++)
            try{
                joint_positions(i) = joint_state.getElementByName(joint_names[i]).position;
            }
            catch(std::exception e){
                throw std::invalid_argument("Kinematic Chain " + chain.getSegment(0).getName() + " to " + chain.getSegment(chain.getNrOfSegments()-1).getName()
                                            + " contains joint " + joint_names[i] + " but this joint is not in joint state vector");
            }

    //// update links
    for(size_t i = 0; i < poses.size(); i++){
        kdl_conversions::RigidBodyState2KDL(poses[i], pose_kdl);

        for(uint j = 0; j < chain.getNrOfSegments(); j++)
            if(chain.segments[j].getName().compare(poses[i].sourceFrame) == 0)
                chain.segments[j] = KDL::Segment(poses[i].sourceFrame, KDL::Joint(KDL::Joint::None), pose_kdl);

        if(poses[i].time > last_update)
            last_update = poses[i].time;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);

    //// compute kinematics
    fk_solver.JntToCart(joint_positions, pose_kdl);
    kdl_conversions::KDL2RigidBodyState(pose_kdl, rigid_body_state);
    rigid_body_state.time = last_update;
    rigid_body_state.sourceFrame = chain.getSegment(chain.getNrOfSegments()-1).getName();
    rigid_body_state.targetFrame = chain.getSegment(0).getName();

    jac_solver.JntToJac(joint_positions, jacobian);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip, so change the reference point to the root frame
    jacobian.changeRefPoint(-pose_kdl.p);
}

} // namespace wbc
