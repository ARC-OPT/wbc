
#include <kdl/utilities/svd_eigen_HH.hpp>
#include "HierarchicalWDLSSolver.hpp"
#include <base/logging.h>
#include <stdexcept>
#include <iostream>

using namespace std;

HierarchicalWDLSSolver::HierarchicalWDLSSolver()
    : epsilon_(1e-9),
      norm_max_(1){

}

bool HierarchicalWDLSSolver::configure(const std::vector<unsigned int> &no_of_constr_per_priority,
                                       const unsigned int no_of_joints){
    if(no_of_joints == 0){
        LOG_ERROR("No of joint variables must be > 0");
        return false;
    }

    if(no_of_constr_per_priority.size() == 0){
        LOG_ERROR("No of priority levels (size of no_of_constr_per_priority) has to be > 0");
        return false;
    }

    for(uint i = 0; i < no_of_constr_per_priority.size(); i++){
        if(no_of_constr_per_priority[i] == 0){
            LOG_ERROR("No of task variables on each priority level must be > 0");
            return false;
        }
    }

    for(uint i = 0; i < no_of_constr_per_priority.size(); i++)
        priorities_.push_back(Priority(no_of_constr_per_priority[i], no_of_joints));

    no_of_joints_ = no_of_joints;
    proj_mat_.resize(no_of_joints_, no_of_joints);
    prev_proj_mat_.resize(no_of_joints_, no_of_joints);
    S_.resize(no_of_joints_);
    S_.setConstant(1.0);
    S_inv_.resize( no_of_joints_, no_of_joints_);
    S_inv_.setZero();
    Damped_S_inv_.resize( no_of_joints_, no_of_joints_);
    S_inv_.setZero();
    V_.resize(no_of_joints_, no_of_joints_);
    V_.setIdentity();
    tmp_.resize(no_of_joints_);
    tmp_.setZero();

    return true;
}

void HierarchicalWDLSSolver::solve(const std::vector<Eigen::MatrixXd> &A_prio,
                                   const std::vector<Eigen::VectorXd> &y_prio,
                                   Eigen::VectorXd &solver_output){

    //Check valid input
    if(solver_output.cols() != no_of_joints_){
        LOG_WARN("Size of output does not match no of joint variables. Will do a resize!");
        solver_output.resize(no_of_joints_);
    }

    if(A_prio.size() != priorities_.size() || y_prio.size() != priorities_.size())
        throw std::invalid_argument("Invalid number of priority levels in input");

    for(uint i = 0; i < priorities_.size(); i++){
        if(A_prio[i].rows() != priorities_[i].no_task_vars_ ||
                A_prio[i].cols() != no_of_joints_ ||
                y_prio[i].rows() != priorities_[i].no_task_vars_)
            throw std::invalid_argument("Invalid size of input variables");
    }

    //Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat_.setIdentity();

    // Loop through all priorities
    for(uint prio = 0; prio < priorities_.size(); prio++){

        //projection of A on the null space of previous priorities
        priorities_[prio].A_projected_ = A_prio[prio] * proj_mat_;

        //Compute svd of A Matrix
        int ret = KDL::svd_eigen_HH(priorities_[prio].A_projected_, priorities_[prio].U_, S_, V_, tmp_);
        if (ret < 0)
            throw std::runtime_error("Unable to perform svd");

        //Compute damping factor based on
        //A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        //Robotic Manipulators through Kinematically Singular Configurations”,
        //Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double sigma_min = S_.block(0,0,min(no_of_joints_, priorities_[prio].no_task_vars_),1).minCoeff();
        double dlambda = 1/norm_max_; //Norm max has to be determined experimentally
        double lambda;
        if(sigma_min <= dlambda/2)
            lambda = dlambda/2;
        else if(sigma_min >= dlambda)
            lambda = 0;
        else
            lambda = sqrt(sigma_min*(dlambda-sigma_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        Damped_S_inv_.setZero();
        for (uint i = 0; i < min(no_of_joints_, priorities_[prio].no_task_vars_); i++)
            Damped_S_inv_(i,i) = (S_(i) / (S_(i) * S_(i) + lambda * lambda));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for (uint i = 0; i < S_.rows(); i++) {
            if(S_(i) < epsilon_) //this is KDL::epsilon of the KDL utilities
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / S_(i);
        }

        // A^# = V * S^# * U^T
        priorities_[prio].A_projected_inv_ =  V_ * S_inv_ * priorities_[prio].U_.transpose();

        // x = A^# * y
        solver_output = priorities_[prio].A_projected_inv_ * y_prio[prio];

    } //priority loop
}
