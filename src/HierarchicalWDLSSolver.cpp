#include "HierarchicalWDLSSolver.hpp"
#include <base/logging.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/time.h>

using namespace std;

namespace wbc{

HierarchicalWDLSSolver::HierarchicalWDLSSolver() :
    nx_(0),
    compute_debug_(false),
    configured_(false),
    epsilon_(1e-9),
    norm_max_(1){

}

bool HierarchicalWDLSSolver::configure(const std::vector<uint> &ny_per_prio,
                                       const unsigned int nx){

    priorities_.clear();

    if(nx == 0){
        LOG_ERROR("No of joint variables must be > 0");
        return false;
    }

    if(ny_per_prio.size() == 0){
        LOG_ERROR("No of priority levels (size of ny_per_prio) has to be > 0");
        return false;
    }

    for(uint i = 0; i < ny_per_prio.size(); i++){
        if(ny_per_prio[i] == 0){
            LOG_ERROR("No of constraint variables on each priority level must be > 0");
            return false;
        }
    }

    ny_per_prio_ = ny_per_prio;

    for(uint prio = 0; prio < ny_per_prio.size(); prio++){
        priorities_.push_back(PriorityDataIntern(ny_per_prio[prio], nx));
        prio_debug_.push_back(PriorityData(ny_per_prio[prio], nx, prio));
    }

    nx_ = nx;
    proj_mat_.resize(nx_, nx_);
    proj_mat_.setIdentity();
    S_.resize(nx_);
    S_.setZero();
    V_.resize(nx_, nx_);
    V_.setIdentity();
    S_inv_.resize( nx_, nx_);
    S_inv_.setZero();
    Damped_S_inv_.resize( nx_, nx_);
    Damped_S_inv_.setZero();
    if(joint_weight_mat_.rows() != nx_ || joint_weight_mat_.cols() != nx_){
        joint_weight_mat_.resize(nx_, nx_);
        joint_weight_mat_.setIdentity();
    }
    Wq_V_.resize(nx_, nx_);
    Wq_V_.setZero();
    Wq_V_S_inv_.resize(nx_, nx_);
    Wq_V_S_inv_.setZero();
    Wq_V_Damped_S_inv_.resize(nx_, nx_);
    Wq_V_Damped_S_inv_.setZero();
    L_.resize(nx_, nx_);
    L_.setZero();

    configured_ = true;

    tmp_.resize(nx_);
    tmp_.setZero();

    return true;
}

void HierarchicalWDLSSolver::solve(const SolverInput& input, Eigen::VectorXd &x){

    //Check valid input
    if(x.rows() != nx_){
        LOG_WARN("Size of output vector does not match number of joint variables. Will do a resize!");
        x.resize(nx_);
    }

    if(input.priorities.size() != priorities_.size())
        throw std::invalid_argument("Invalid number of priority levels in input");
    for(uint i = 0; i < priorities_.size(); i++){
        if(input.priorities[i].A.rows() != priorities_[i].ny_ ||
                input.priorities[i].A.cols() != nx_ ||
                input.priorities[i].y_ref.rows() != priorities_[i].ny_){
            LOG_ERROR("Expected input size: A: %i x %i, y: %i x 1, actual input: A: %i x %i, y: %i x 1",
                      priorities_[i].ny_, nx_, priorities_[i].ny_, input.priorities[i].A.rows(), input.priorities[i].A.cols(), input.priorities[i].y_ref.rows());
            throw std::invalid_argument("Invalid size of input variables");
        }
    }

    //Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat_.setIdentity();
    x.setZero();

    //////// Loop through all priorities

    for(uint prio = 0; prio < priorities_.size(); prio++){

        base::Time start = base::Time::now(), loop_start = base::Time::now();

        setTaskWeights(input.priorities[prio].Wy, prio);

        priorities_[prio].y_comp_.setZero();

        //Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities_[prio].y_comp_ = input.priorities[prio].y_ref - input.priorities[prio].A*x;

        //projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        //For the first priority P == Identity
        priorities_[prio].A_proj_ = input.priorities[prio].A * proj_mat_;

        double comp_time_proj = (base::Time::now() - start).toSeconds();
        start = base::Time::now();

        //Compute weighted, projected mat: A_proj_w = Wy * A_proj * Wq^-1
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication

        for(uint i = 0; i < priorities_[prio].ny_; i++)
            priorities_[prio].A_proj_w_.row(i) = priorities_[prio].task_weight_mat_(i,i) * priorities_[prio].A_proj_.row(i);

        for(uint i = 0; i < nx_; i++)
            priorities_[prio].A_proj_w_.col(i) = joint_weight_mat_(i,i) * priorities_[prio].A_proj_w_.col(i);

        double comp_time_weighting = (base::Time::now() - start).toSeconds();
        start = base::Time::now();

        if(svd_method_ == svd_eigen)
        {
            //Compute svd of A Matrix: A = U*Sigma*V^T, where Sigma contains the singular values on its main diagonal
            priorities_[prio].svd_.compute(priorities_[prio].A_proj_w_, Eigen::ComputeFullV | Eigen::ComputeFullU);

            V_ = priorities_[prio].svd_.matrixV();
            uint ns = priorities_[prio].svd_.singularValues().size(); //No of singular values

            //Entries of S that are not singular values will be zero
            S_.setZero();
            S_.block(0,0,ns,1) = priorities_[prio].svd_.singularValues();

            //U output of svd will have different size than required, copy only the first ns columns. They contain the relevant Eigenvectors
            priorities_[prio].U_.block(0,0, priorities_[prio].ny_, ns) =
                    priorities_[prio].svd_.matrixU().block(0,0,priorities_[prio].ny_, ns);
        }
        else if(svd_method_ == svd_kdl)
        {
            KDL::svd_eigen_HH(priorities_[prio].A_proj_w_, priorities_[prio].U_, S_, V_, tmp_);
        }
        else{
            LOG_ERROR("Invalid svd method: %i", svd_method_);
            throw std::invalid_argument("Invalid svd method");
        }

        double comp_time_svd = (base::Time::now() - start).toSeconds();
        start = base::Time::now();

        //Compute damping factor based on
        //A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        //Robotic Manipulators through Kinematically Singular Configurations”,
        //Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double s_min = S_.block(0,0,min(nx_, priorities_[prio].ny_),1).minCoeff();
        double damping;
        if(s_min <= (1/norm_max_)/2)
            damping = (1/norm_max_)/2;
        else if(s_min >= (1/norm_max_))
            damping = 0;
        else
            damping = sqrt(s_min*((1/norm_max_)-s_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        Damped_S_inv_.setZero();
        for (uint i = 0; i < min(nx_, priorities_[prio].ny_); i++)
            Damped_S_inv_(i,i) = (S_(i) / (S_(i) * S_(i) + damping * damping));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < S_.rows(); i++){
            if(S_(i) < epsilon_)
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / S_(i);
        }

        // A^# = Wq^-1 * V * S^# * U^T * Wy
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication (saves a lot of computation!)
        priorities_[prio].u_t_task_weight_mat_ = priorities_[prio].U_.transpose();
        for(uint i = 0; i < priorities_[prio].ny_; i++)
            priorities_[prio].u_t_task_weight_mat_.col(i) = priorities_[prio].u_t_task_weight_mat_.col(i) * priorities_[prio].task_weight_mat_(i,i);

        for(uint i = 0; i < nx_; i++)
            Wq_V_.row(i) = joint_weight_mat_(i,i) * V_.row(i);

        for(uint i = 0; i < nx_; i++)
            Wq_V_S_inv_.col(i) = Wq_V_.col(i) * S_inv_(i,i);
        for(uint i = 0; i < nx_; i++)
            Wq_V_Damped_S_inv_.col(i) = Wq_V_.col(i) * Damped_S_inv_(i,i);

        priorities_[prio].A_proj_inv_wls_ = Wq_V_S_inv_ * priorities_[prio].u_t_task_weight_mat_; //Normal Inverse with weighting
        priorities_[prio].A_proj_inv_wdls_ = Wq_V_Damped_S_inv_ * priorities_[prio].u_t_task_weight_mat_; //Damped inverse with weighting

        // x = x + A^# * y
        x += priorities_[prio].A_proj_inv_wdls_ * priorities_[prio].y_comp_;

        // Compute projection matrix for the next priority. Use here the undamped inverse to have a correct solution
        proj_mat_ -= priorities_[prio].A_proj_inv_wls_ * priorities_[prio].A_proj_;

        double comp_time_inv = (base::Time::now() - start).toSeconds();

        if(compute_debug_)
        {
            prio_debug_[prio].y_des = input.priorities[prio].y_ref;
            prio_debug_[prio].y_solution = input.priorities[prio].A * x;
            prio_debug_[prio].singular_vals = S_;
            prio_debug_[prio].manipulability = sqrt( (priorities_[prio].A_proj_w_ * priorities_[prio].A_proj_w_.transpose()).determinant() );
            prio_debug_[prio].sqrt_wbc_err = sqrt((prio_debug_[prio].y_des - prio_debug_[prio].y_solution).norm());
            prio_debug_[prio].damping = damping;
            prio_debug_[prio].proj_time = comp_time_proj;
            prio_debug_[prio].weighting_time = comp_time_weighting;
            prio_debug_[prio].svd_time = comp_time_svd;
            prio_debug_[prio].compute_inverse_time = comp_time_inv;
            prio_debug_[prio].total_time = (base::Time::now() - loop_start).toSeconds();

            //Get maximum and minimum singular value
            double max_singular_val = 0;
            double min_singular_val = 1e10;
            for(uint i = 0 ; i < S_.rows(); i++)
            {
                if(S_(i) > max_singular_val)
                    max_singular_val = S_(i);
                if(S_(i) > 0 && S_(i) < min_singular_val)
                    min_singular_val = S_(i);
            }

            prio_debug_[prio].max_singular_val = max_singular_val;
            if(min_singular_val == 1e10)
                min_singular_val = 0;
            prio_debug_[prio].min_singular_val = min_singular_val;
        }
    } //priority loop

    ///////////////
}

void HierarchicalWDLSSolver::setJointWeights(const Eigen::VectorXd& weights){
    if(weights.size() == nx_){
        joint_weight_mat_.setZero();
        for(uint i = 0; i < nx_; i++)
            joint_weight_mat_(i,i) = sqrt(weights(i));
    }
    else{
        LOG_ERROR("Cannot set joint weights. Size of weight vector %i but should be %i", weights.size(), nx_);
        throw std::invalid_argument("Invalid Joint weight vector size");
    }
}

void HierarchicalWDLSSolver::   setTaskWeights(const Eigen::VectorXd& weights, const uint prio){
    if(prio >= priorities_.size()){
        LOG_ERROR("Cannot set constraint weights. Given Priority is %i but number of priority levels is %i", prio, priorities_.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(priorities_[prio].ny_ != weights.size()){
        LOG_ERROR("Cannot set constraint weights. Size of weight mat is %i but should be %i", weights.size(), priorities_[prio].ny_);
        throw std::invalid_argument("Invalid Weight vector size");
    }
    priorities_[prio].task_weight_mat_.setZero();
    for(uint i = 0; i < priorities_[prio].ny_; i++)
        priorities_[prio].task_weight_mat_(i,i) = sqrt(weights(i));
}
}
