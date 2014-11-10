#include "KinematicChainKDL.hpp"
#include <base/Logging.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain& chain)
{
    chain_ = chain;
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);

    q_.resize(chain.getNrOfJoints());
    q_dot_.resize(chain.getNrOfJoints());
    q_dot_dot_.resize(chain.getNrOfJoints());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names_.push_back(joint.getName());
    }

    //Create task Frame, name will be same as last name in kin chain
    tf = new TaskFrameKDL(chain.getSegment(chain.getNrOfSegments()-1).getName(), joint_names_);
}

KinematicChainKDL::~KinematicChainKDL(){
    delete pos_fk_solver_;
    delete jac_solver_;
    delete tf;
}

void KinematicChainKDL::update(const base::samples::Joints &status){

    //Update joint variables
    for(uint i = 0; i < joint_names_.size(); i++){
        size_t idx;
        try{
            idx = status.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Kin. Chain of task frame %s has joint %s, but this joint is not in joint state vector", tf->tf_name.c_str(), joint_names_[i].c_str());
            throw e;
        }
        q_(i) = status[idx].position;
        q_dot_(i) = status[idx].speed;
        q_dot_dot_(i) = status[idx].effort;
    }

    //Compute FK
    pos_fk_solver_->JntToCart(q_, tf->pose);

    //Compute Jacobian
    jac_solver_->JntToJac(q_, tf->jac);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    tf->jac.changeRefPoint(-tf->pose.p);

}
}
