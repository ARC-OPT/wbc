#include "SubTask.hpp"
#include <stdexcept>

SubTask::SubTask(const KDL::Chain &chain, const uint no_task_variables, const uint priority){
    chain_ = chain;
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);
    no_task_variables_ = no_task_variables;
    priority_ = priority;

    q_.resize(chain.getNrOfJoints());
    q_dot_.resize(chain.getNrOfJoints());
    q_dot_dot_.resize(chain.getNrOfJoints());
}

SubTask::~SubTask(){
    delete pos_fk_solver_;
    delete jac_solver_;
}

void SubTask::Update(){
    //Compute FK
    pos_fk_solver_->JntToCart(q_, pose_);

    //Compute Jacobian
    jac_solver_->JntToJac(q_dot_, jac_);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jac_.changeRefPoint(-pose_.p);

    //TODO: Add dynamic equations here
}

