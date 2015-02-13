#include "KinematicChainKDL.hpp"
#include <base/Logging.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain& chain, const std::string &id)
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
    tf = new TaskFrameKDL(id, joint_names_);
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
            LOG_ERROR("Kin. Chain of task frame %s has joint %s, but this joint is not in joint state vector", tf->tf_name_.c_str(), joint_names_[i].c_str());
            throw e;
        }
        q_(i) = status[idx].position;
        q_dot_(i) = status[idx].speed;
        q_dot_dot_(i) = status[idx].effort;
    }

    //Compute FK
    pos_fk_solver_->JntToCart(q_, tf->pose_);

    //Compute Jacobian
    jac_solver_->JntToJac(q_, tf->jac_);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    tf->jac_.changeRefPoint(-tf->pose_.p);
}

KinematicChainKDLDyn::KinematicChainKDLDyn(const KDL::Chain &chain, const Eigen::Vector3d &gravity, const std::string &id) :
    KinematicChainKDL(chain, id)
{
    dyn_param_solver_ = new KDL::ChainDynParam(chain, KDL::Vector(gravity(0), gravity(1), gravity(2)));
    jnt_inertia_.resize(chain.getNrOfJoints());
    jnt_gravity_.resize(chain.getNrOfJoints());
    jnt_coriolis_.resize(chain.getNrOfJoints());

}

KinematicChainKDLDyn::~KinematicChainKDLDyn()
{
    delete dyn_param_solver_;
}

void KinematicChainKDLDyn::update(const base::samples::Joints &status)
{
    KinematicChainKDL::update(status);

    dyn_param_solver_->JntToMass(q_, jnt_inertia_);
    tf->jnt_inertia_ = jnt_inertia_.data;

    dyn_param_solver_->JntToGravity(q_, jnt_gravity_);
    tf->jnt_gravity_ = jnt_gravity_.data;

    dyn_param_solver_->JntToCoriolis(q_, q_dot_, jnt_coriolis_);
    tf->jnt_coriolis_ = jnt_coriolis_.data;
}
}
