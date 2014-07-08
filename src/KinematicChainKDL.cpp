#include "KinematicChainKDL.hpp"
#include <base/Logging.hpp>

namespace wbc{

KinematicChainKDL::KinematicChainKDL(const KDL::Chain& chain, const JointIndexMap& joint_index_map)
{
    chain_ = chain;
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);

    jac_kdl_ = KDL::Jacobian(chain_.getNrOfJoints());
    jac_kdl_.data.setZero(6,chain_.getNrOfJoints());

    jac_robot_kdl_ = KDL::Jacobian(joint_index_map.size());
    jac_robot_kdl_.data.setZero(6, joint_index_map.size());

    q_.resize(chain.getNrOfJoints());
    q_dot_.resize(chain.getNrOfJoints());
    q_dot_dot_.resize(chain.getNrOfJoints());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names_.push_back(joint.getName());
    }

    joint_index_map_ = joint_index_map;
}

KinematicChainKDL::~KinematicChainKDL(){
    delete pos_fk_solver_;
    delete jac_solver_;
}

void KinematicChainKDL::update(const base::samples::Joints &status){

    //Update joint variables
    for(uint i = 0; i < joint_names_.size(); i++){
        size_t idx;
        try{
            idx = status.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Kin. Chain of task frame %s has joint %s, but this joint is not in joint state vector", tf_name.c_str(), joint_names_[i].c_str());
            throw e;
        }
        q_(i) = status[idx].position;
        q_dot_(i) = status[idx].speed;
        q_dot_dot_(i) = status[idx].effort;
    }

    //Compute FK
    pos_fk_solver_->JntToCart(q_, pose_kdl_);

    //Compute Jacobian
    jac_solver_->JntToJac(q_, jac_kdl_);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jac_kdl_.changeRefPoint(-pose_kdl_.p);

    //IMPORTANT: Fill in columns of Jacobian into the correct place of the full robot Jacobian using the joint_index_map
    for(uint i = 0; i < joint_names_.size(); i++){
        uint idx = joint_index_map_[joint_names_[i]];
        jac_robot_kdl_.setColumn(idx, jac_kdl_.getColumn(i));
    }

    kdl_conversions::KDL2RigidBodyState(pose_kdl_, pose);
    jac = jac_robot_kdl_.data;
}
}
