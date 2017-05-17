#include "KinematicChainKDL.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl_conversions/KDLConversions.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(KDL::Chain chain) :
    chain(chain){

    joint_positions.resize(chain.getNrOfJoints());
    jacobian = KDL::Jacobian(chain.getNrOfJoints());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        const KDL::Joint &joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }

    fk_solver  = new KDL::ChainFkSolverPos_recursive(chain);
    jac_solver = new KDL::ChainJntToJacSolver(chain);
}

KinematicChainKDL::~KinematicChainKDL(){
    delete fk_solver;
    delete jac_solver;
}

void KinematicChainKDL::update(const base::samples::Joints &joint_state, const std::vector<base::samples::RigidBodyState> &poses){

    //// update Joints
    ///
    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            joint_positions(i) = joint_state.getElementByName(joint_names[i]).position;
        }
        catch(std::exception e){
            throw std::invalid_argument("Kinematic Chain " + chain.getSegment(0).getName() + " to " + chain.getSegment(chain.getNrOfSegments()-1).getName()
                                        + " contains joint " + joint_names[i] + " but this joint is not in joint state vector");
        }
    }

    //// update links
    ///
    bool recreate_solvers = false;
    for(size_t i = 0; i < poses.size(); i++){
        kdl_conversions::RigidBodyState2KDL(poses[i], pose_kdl);

        for(uint j = 0; j < chain.getNrOfSegments(); j++)
            if(chain.segments[j].getName().compare(poses[i].sourceFrame) == 0){
                chain.segments[j] = KDL::Segment(poses[i].sourceFrame, KDL::Joint(KDL::Joint::None), pose_kdl);
                recreate_solvers = true;
            }
    }
    if(recreate_solvers){
        delete fk_solver;
        delete jac_solver;
        fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
        jac_solver = new KDL::ChainJntToJacSolver(chain);
    }

    //// compute kinematics
    ///
    fk_solver->JntToCart(joint_positions, pose_kdl);
    kdl_conversions::KDL2RigidBodyState(pose_kdl, rigid_body_state);

    jac_solver->JntToJac(joint_positions, jacobian);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jacobian.changeRefPoint(-pose_kdl.p);
}

} // namespace wbc
