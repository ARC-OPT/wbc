#include "HierarchicalWDLSSolver.hpp"
#include <base/logging.h>
#include <stdexcept>
#include <iostream>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/time.h>

using namespace std;

namespace wbc{

HierarchicalWDLSSolver::HierarchicalWDLSSolver() :
    nx_(0),
    configured_(false),
    epsilon_(1e-9),
    norm_max_(1){

}

bool HierarchicalWDLSSolver::configure(const std::vector<int> &ny_per_prio,
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
    if(column_weight_mat_.rows() != nx_ || column_weight_mat_.cols() != nx_){
        column_weight_mat_.resize(nx_, nx_);
        column_weight_mat_.setIdentity();
        column_weights_.resize(nx_);
        column_weights_.setOnes(nx_);
    }
    Wq_V_.resize(nx_, nx_);
    Wq_V_.setZero();
    Wq_V_S_inv_.resize(nx_, nx_);
    Wq_V_S_inv_.setZero();
    Wq_V_Damped_S_inv_.resize(nx_, nx_);
    Wq_V_Damped_S_inv_.setZero();

    configured_ = true;

    tmp_.resize(nx_);
    tmp_.setZero();

    return true;
}

void HierarchicalWDLSSolver::solve(const std::vector<Eigen::MatrixXd> &A,
                                   const std::vector<Eigen::VectorXd> &Wy,
                                   const std::vector<Eigen::VectorXd> &y_ref,
                                   const Eigen::VectorXd &Wx,
                                   Eigen::VectorXd &x){

    //Check valid input

    if(A.size() != priorities_.size() ||
       Wy.size() != priorities_.size() ||
       y_ref.size() != priorities_.size()){
        LOG_ERROR("Number of priorities in solver: %i, Size of input vectors: A-vector: %i, Wy-vector: %i, y_ref-vector: %i", priorities_.size(), A.size(), Wy.size(), y_ref.size());
        throw std::invalid_argument("Invalid solver input");
    }

    if(x.rows() != nx_){
        LOG_WARN("Size of output vector does not match number of joint variables. Will do a resize!");
        x.resize(nx_);
    }

    for(uint i = 0; i < priorities_.size(); i++)
    {
        if(A[i].rows() != priorities_[i].ny_ ||
           A[i].cols() != nx_ ||
           y_ref[i].rows() != priorities_[i].ny_){
            LOG_ERROR("Expected input size on priority level %i: A: %i x %i, y: %i x 1, actual input: A: %i x %i, y: %i x 1",
                      i, priorities_[i].ny_, nx_, priorities_[i].ny_, A[i].rows(), A[i].cols(), y_ref[i].rows());
            throw std::invalid_argument("Invalid size of input variables");
        }
    }

    setColumnWeights(Wx);

    //Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat_.setIdentity();
    x.setZero();

    //////// Loop through all priorities

    for(uint prio = 0; prio < priorities_.size(); prio++){

        setRowWeights(Wy[prio], prio);

        priorities_[prio].y_comp_.setZero();

        //Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities_[prio].y_comp_ = y_ref[prio] - A[prio]*x;

        //projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        //For the first priority P == Identity
        priorities_[prio].A_proj_ = A[prio] * proj_mat_;

        //Compute weighted, projected mat: A_proj_w = Wy * A_proj * Wq^-1
        //Since the weight matrices are diagonal, there is no need for full matrix multiplication

        for(uint i = 0; i < priorities_[prio].ny_; i++)
            priorities_[prio].A_proj_w_.row(i) = priorities_[prio].row_weight_mat_(i,i) * priorities_[prio].A_proj_.row(i);

        for(uint i = 0; i < nx_; i++)
            priorities_[prio].A_proj_w_.col(i) = column_weight_mat_(i,i) * priorities_[prio].A_proj_w_.col(i);

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

        //Compute damping factor based on
        //A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        //Robotic Manipulators through Kinematically Singular Configurations”,
        //Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double s_min = S_.block(0,0,min(nx_, priorities_[prio].ny_),1).minCoeff();
        if(s_min <= (1/norm_max_)/2)
            priorities_[prio].damping_ = (1/norm_max_)/2;
        else if(s_min >= (1/norm_max_))
            priorities_[prio].damping_ = 0;
        else
            priorities_[prio].damping_ = sqrt(s_min*((1/norm_max_)-s_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        Damped_S_inv_.setZero();
        for (uint i = 0; i < min(nx_, priorities_[prio].ny_); i++)
            Damped_S_inv_(i,i) = (S_(i) / (S_(i) * S_(i) + priorities_[prio].damping_ * priorities_[prio].damping_));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < S_.rows(); i++){
            if(S_(i) < epsilon_)
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / S_(i);
        }

        // A^# = Wq^-1 * V * S^# * U^T * Wy
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication (saves a lot of computation!)
        priorities_[prio].u_t_row_weight_mat_ = priorities_[prio].U_.transpose();
        for(uint i = 0; i < priorities_[prio].ny_; i++)
            priorities_[prio].u_t_row_weight_mat_.col(i) = priorities_[prio].u_t_row_weight_mat_.col(i) * priorities_[prio].row_weight_mat_(i,i);

        for(uint i = 0; i < nx_; i++)
            Wq_V_.row(i) = column_weight_mat_(i,i) * V_.row(i);

        for(uint i = 0; i < nx_; i++)
            Wq_V_S_inv_.col(i) = Wq_V_.col(i) * S_inv_(i,i);
        for(uint i = 0; i < nx_; i++)
            Wq_V_Damped_S_inv_.col(i) = Wq_V_.col(i) * Damped_S_inv_(i,i);

        priorities_[prio].A_proj_inv_wls_ = Wq_V_S_inv_ * priorities_[prio].u_t_row_weight_mat_; //Normal Inverse with weighting
        priorities_[prio].A_proj_inv_wdls_ = Wq_V_Damped_S_inv_ * priorities_[prio].u_t_row_weight_mat_; //Damped inverse with weighting

        // x = x + A^# * y
        priorities_[prio].x_prio_ = priorities_[prio].A_proj_inv_wdls_ * priorities_[prio].y_comp_;
        x += priorities_[prio].x_prio_;

        // Compute projection matrix for the next priority. Use here the undamped inverse to have a correct solution
        proj_mat_ -= priorities_[prio].A_proj_inv_wls_ * priorities_[prio].A_proj_;

        //store eigenvalues for this priority
        priorities_[prio].singular_values_.setZero();
        for(uint i = 0; i < min(nx_, priorities_[prio].ny_); i++)
            priorities_[prio].singular_values_(i) = S_(i);

    } //priority loop

    ///////////////
}

void HierarchicalWDLSSolver::setColumnWeights(const Eigen::VectorXd& weights){
    if(weights.size() == nx_){
        column_weights_ = weights;
        column_weight_mat_.setZero();
        for(uint i = 0; i < nx_; i++)
            column_weight_mat_(i,i) = sqrt(weights(i));
    }
    else{
        LOG_ERROR("Cannot set joint weights. Size of weight vector %i but should be %i", weights.size(), nx_);
        throw std::invalid_argument("Invalid Joint weight vector size");
    }
}

void HierarchicalWDLSSolver::setRowWeights(const Eigen::VectorXd& weights, const uint prio){
    if(prio >= priorities_.size()){
        LOG_ERROR("Cannot set constraint weights. Given Priority is %i but number of priority levels is %i", prio, priorities_.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(priorities_[prio].ny_ != weights.size()){
        LOG_ERROR("Cannot set constraint weights. Size of weight mat is %i but should be %i", weights.size(), priorities_[prio].ny_);
        throw std::invalid_argument("Invalid Weight vector size");
    }
    priorities_[prio].row_weight_mat_.setZero();
    for(uint i = 0; i < priorities_[prio].ny_; i++)
        priorities_[prio].row_weight_mat_(i,i) = sqrt(weights(i));
}

const HierarchicalWDLSSolver::PriorityDataIntern& HierarchicalWDLSSolver::getPriorityData(const uint prio)
{
    if(prio < 0 || prio >= priorities_.size())
    {
        LOG_ERROR("Invalid priority: %i", prio);
        throw std::invalid_argument("Invalid priority");
    }
    return priorities_[prio];
}
}
