#include "SubTask.hpp"
#include <stdexcept>

SubTask::SubTask(const KDL::Chain &chain){
    chain_ = chain;
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);
}

SubTask::~SubTask(){
    delete pos_fk_solver_;
    delete jac_solver_;
}

void SubTask::Update(const KDL::JntArray &q, const KDL::JntArray& q_dot, const KDL::JntArray &q_dot_dot){
    if(q.columns() != chain_.getNrOfJoints())
        throw std::invalid_argument("Invalid number of input joint angles");

    if(q_dot.columns() != chain_.getNrOfJoints())
        throw std::invalid_argument("Invalid number of input joint velocities");

    if(q_dot_dot.columns() != chain_.getNrOfJoints())
        throw std::invalid_argument("Invalid number of input joint accelerations");

    //Compute FK
    pos_fk_solver_->JntToCart(q, pose_);

    //Compute Jacobian
    jac_solver_->JntToJac(q_dot, jac_);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jac_.changeRefPoint(-pose_.p);

    //TODO: Add dynamic equations here
}

