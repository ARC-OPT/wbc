#include "HierarchicalWDLSSolver.hpp"
#include <base/logging.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <kdl/utilities/svd_eigen_HH.hpp>

using namespace std;

namespace wbc{

HierarchicalWDLSSolver::HierarchicalWDLSSolver() :
    nx_(0),
    damping_(0),
    configured_(false),
    joint_weight_mat_is_diagonal_(true),
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
            LOG_ERROR("No of task variables on each priority level must be > 0");
            return false;
        }
    }

    for(uint prio = 0; prio < ny_per_prio.size(); prio++)
        priorities_.push_back(Priority(ny_per_prio[prio], nx));


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
    joint_weight_mat_.resize(nx_, nx_);
    joint_weight_mat_.setIdentity();
    Wq_V_.resize(nx_, nx_);
    Wq_V_.setZero();
    Wq_V_S_inv_.resize(nx_, nx_);
    Wq_V_S_inv_.setZero();
    Wq_V_Damped_S_inv_.resize(nx_, nx_);
    Wq_V_Damped_S_inv_.setZero();
    L_.resize(nx_, nx_);
    L_.setZero();

    damping_ = 0;
    joint_weight_mat_is_diagonal_ = true;
    configured_ = true;

    tmp_.resize(nx_);
    tmp_.setZero();

    return true;
}

void HierarchicalWDLSSolver::solve(const std::vector<Eigen::MatrixXd> &A,
                                   const std::vector<Eigen::VectorXd> &y,
                                   Eigen::VectorXd &x){
    //Check valid input
    if(x.rows() != nx_){
        LOG_WARN("Size of output vector does not match number of joint variables. Will do a resize!");
        x.resize(nx_);
    }

    if(A.size() != priorities_.size() || y.size() != priorities_.size())
        throw std::invalid_argument("Invalid number of priority levels in input");
    for(uint i = 0; i < priorities_.size(); i++){
        if(A[i].rows() != priorities_[i].ny_ ||
                A[i].cols() != nx_ ||
                y[i].rows() != priorities_[i].ny_){
            LOG_ERROR("Expected input size: A: %i x %i, y: %i x 1, actual input: A: %i x %i, y: %i x 1",
                      priorities_[i].ny_, nx_, priorities_[i].ny_, A[i].rows(), A[i].cols(), y[i].rows());
            throw std::invalid_argument("Invalid size of input variables");
        }
    }

    //Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat_.setIdentity();
    x.setZero();

    //////// Loop through all priorities

    for(uint prio = 0; prio < priorities_.size(); prio++){

        priorities_[prio].comp_times_.init();

        priorities_[prio].y_comp_.setZero();

        //Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities_[prio].y_comp_ = y[prio] - A[prio]*x;

        //projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        //For the first priority P == Identity
        priorities_[prio].A_proj_ = A[prio] * proj_mat_;

        priorities_[prio].comp_times_.set_proj_time(base::Time::now());

        //Compute weighted, projected mat: A_proj_w = Wy * A_proj * Wq^-1
        // If the weight matrices are diagonal, there is no need for full matrix multiplication

        if(priorities_[prio].task_weight_mat_is_diagonal_){
            for(uint i = 0; i < priorities_[prio].ny_; i++)
                priorities_[prio].A_proj_w_.row(i) = priorities_[prio].task_weight_mat_(i,i) * priorities_[prio].A_proj_.row(i);
        }
        else
            priorities_[prio].A_proj_w_ = priorities_[prio].task_weight_mat_ *  priorities_[prio].A_proj_;

        if(joint_weight_mat_is_diagonal_){
            for(uint i = 0; i < nx_; i++)
                priorities_[prio].A_proj_w_.col(i) = joint_weight_mat_(i,i) * priorities_[prio].A_proj_w_.col(i);
        }
        else
            priorities_[prio].A_proj_w_ = priorities_[prio].A_proj_w_ * joint_weight_mat_;

        priorities_[prio].comp_times_.set_weighting_time(base::Time::now());

        if(svd_method_ == svd_eigen)
        {
            //Compute svd of A Matrix: A = U*Sigma*V^T, where Sigma contains the singular values on its main diagonal
            priorities_[prio].svd_.compute(priorities_[prio].A_proj_w_, Eigen::ComputeFullV | Eigen::ComputeFullU);

            priorities_[prio].comp_times_.set_svd_time(base::Time::now());

            V_ = priorities_[prio].svd_.matrixV();
            uint ns = priorities_[prio].svd_.singularValues().size(); //No of singular values

            //Entries of S that are not singular values will be zero
            S_.setZero();
            S_.block(0,0,ns,1) = priorities_[prio].svd_.singularValues();

            //U output of svd will have different size than required, copy only the first ns columns. They contain the relevant Eigenvectors
            priorities_[prio].U_.block(0,0, priorities_[prio].ny_, ns) =
                    priorities_[prio].svd_.matrixU().block(0,0,priorities_[prio].ny_, ns);
        }
        else
        {
            KDL::svd_eigen_HH(priorities_[prio].A_proj_w_, priorities_[prio].U_, S_, V_, tmp_);
            priorities_[prio].comp_times_.set_svd_time(base::Time::now());
        }

        //Compute damping factor based on
        //A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        //Robotic Manipulators through Kinematically Singular Configurations”,
        //Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double s_min = S_.block(0,0,min(nx_, priorities_[prio].ny_),1).minCoeff();
        if(s_min <= (1/norm_max_)/2)
            damping_ = (1/norm_max_)/2;
        else if(s_min >= (1/norm_max_))
            damping_ = 0;
        else
            damping_ = sqrt(s_min*((1/norm_max_)-s_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        Damped_S_inv_.setZero();
        for (uint i = 0; i < min(nx_, priorities_[prio].ny_); i++)
            Damped_S_inv_(i,i) = (S_(i) / (S_(i) * S_(i) + damping_ * damping_));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < S_.rows(); i++){
            if(S_(i) < epsilon_)
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / S_(i);
        }

        // A^# = Wq^-1 * V * S^# * U^T * Wy
        // If the weight matrices are diagonal, there is no need for full matrix multiplication (saves a lot of computation!)

        if(priorities_[prio].task_weight_mat_is_diagonal_){
            priorities_[prio].u_t_task_weight_mat_ = priorities_[prio].U_.transpose();
            for(uint i = 0; i < priorities_[prio].ny_; i++)
                priorities_[prio].u_t_task_weight_mat_.col(i) = priorities_[prio].u_t_task_weight_mat_.col(i) * priorities_[prio].task_weight_mat_(i,i);
        }
        else
            priorities_[prio].u_t_task_weight_mat_ = priorities_[prio].U_.transpose() * priorities_[prio].task_weight_mat_;

        if(joint_weight_mat_is_diagonal_){
            for(uint i = 0; i < nx_; i++)
                Wq_V_.row(i) = joint_weight_mat_(i,i) * V_.row(i);
        }
        else
            Wq_V_ = joint_weight_mat_ * V_;

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

        priorities_[prio].comp_times_.set_compute_inverse_time(base::Time::now());

    } //priority loop

    ///////////////
}

bool HierarchicalWDLSSolver::isMatDiagonal(const Eigen::MatrixXd& mat)
{
    //Check if new matrix is diagonal
    bool is_diagonal = true;
    for(uint i = 0; i < mat.rows(); i++){
        for(uint j = 0; j < mat.cols(); j++){
            if(i != j){
                if(mat(i,j) != 0)
                    is_diagonal = false;
            }
        }
    }
    return is_diagonal;
}

void HierarchicalWDLSSolver::setJointWeights(const Eigen::MatrixXd& weights){
    if(weights.rows() == nx_ &&
            weights.cols() == nx_){

        //Check if new matrix is diagonal
        joint_weight_mat_is_diagonal_ = isMatDiagonal(weights);
        if(joint_weight_mat_is_diagonal_){
            joint_weight_mat_.setZero();
            for(uint i = 0; i < nx_; i++){
                if(weights(i,i) == 0)
                    throw std::invalid_argument("Joint weights are not allowed to be zero! Note that a high joint weight will make a joint move less");
                else
                    joint_weight_mat_(i,i) = sqrt(1/weights(i,i));
            }
        }
        else{
            Eigen::LLT<Eigen::MatrixXd> llt(weights); // compute the Cholesky decomposition of A
            L_ = llt.matrixL(); // retrieve factor L  in the decomposition
            joint_weight_mat_ =  L_.transpose().inverse();
        }
    }
    else{
        LOG_ERROR("Cannot set joint weights. Size of weight matrix is %i x %i but should be %i x %i", weights.rows(), weights.cols(), nx_, nx_);
        throw std::invalid_argument("Invalid Joint weight vector size");
    }
}

void HierarchicalWDLSSolver::setTaskWeights(const Eigen::MatrixXd& weights, const uint prio){
    if(prio >= priorities_.size()){
        LOG_ERROR("Cannot set task weights. Given Priority is %i but number of priority levels is %i", prio, priorities_.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(priorities_[prio].ny_ != weights.rows() ||
            priorities_[prio].ny_ != weights.cols()){
        LOG_ERROR("Cannot set task weights. Size of weight mat is %i x %i but should be %i x %i", weights.rows(), weights.cols(), priorities_[prio].ny_, priorities_[prio].ny_);
        throw std::invalid_argument("Invalid Weight vector size");
    }

    //Check if new matrix is diagonal
    priorities_[prio].task_weight_mat_is_diagonal_ = isMatDiagonal(weights);

    if(priorities_[prio].task_weight_mat_is_diagonal_){
        priorities_[prio].task_weight_mat_.setZero();
        for(uint i = 0; i < priorities_[prio].ny_; i++)
            priorities_[prio].task_weight_mat_(i,i) = sqrt(weights(i,i));
    }
    else{
        // compute the Cholesky decomposition of weights
        Eigen::LLT<Eigen::MatrixXd> llt(weights);
        priorities_[prio].task_weight_mat_ = llt.matrixL().transpose();
    }
}
}
