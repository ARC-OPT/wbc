
#include <kdl/utilities/svd_eigen_HH.hpp>
#include "HierarchicalWDLSSolver.hpp"
#include <base/logging.h>
#include <stdexcept>
#include <iostream>

using namespace std;

HierarchicalWDLSSolver::HierarchicalWDLSSolver() : HierarchicalSolver(),
    epsilon_(1e-9),
    norm_max_(1){

}

bool HierarchicalWDLSSolver::configure(const std::vector<unsigned int> &ny_per_prio,
                                       const unsigned int nx){
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
    S_.resize(nx_);
    S_.setZero();
    S_inv_.resize( nx_, nx_);
    S_inv_.setZero();
    Damped_S_inv_.resize( nx_, nx_);
    S_inv_.setZero();
    V_.resize(nx_, nx_);
    V_.setIdentity();
    tmp_.resize(nx_);
    tmp_.setZero();
    joint_weight_mat_.resize(nx_, nx_);

    //Set Joint weights to default (all 1)
    Eigen::VectorXd joint_weights;
    joint_weights.resize(nx_);
    joint_weights.setConstant(1);
    setJointWeights(joint_weights);

    //Set all task weights to default (all 1)
    for(uint prio = 0; prio < ny_per_prio.size(); prio++){
        Eigen::VectorXd task_weights;
        task_weights.resize(priorities_[prio].ny_);
        task_weights.setConstant(1);
        setTaskWeights(task_weights, prio);
    }


    return true;
}

void HierarchicalWDLSSolver::solve(const std::vector<Eigen::MatrixXd> &A,
                                   const std::vector<Eigen::VectorXd> &y,
                                   Eigen::VectorXd &x){
    //Check valid input
    if(x.cols() != nx_){
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

        priorities_[prio].y_comp_.setZero();

        //Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities_[prio].y_comp_ = y[prio] - A[prio]*x;

        //projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        //For the first priority P == Identity
        priorities_[prio].A_proj_ = A[prio] * proj_mat_;

        //Compute weighted projection mat: A_proj_w = Wy * A_proj * Wq^-1
        Eigen::MatrixXd tmp(nx_, nx_);
        tmp = priorities_[prio].A_proj_ * joint_weight_mat_;
        priorities_[prio].A_proj_w_ = priorities_[prio].task_weight_mat_ * tmp;//priorities_[prio].A_proj_ * joint_weight_mat_;

        //Compute svd of A Matrix TODO: Check the Eigen Version of SVD to get rid of KDL dependency here
        /*priorities_[prio].svd_.compute(priorities_[prio].A_proj_w_, ComputeFullV | ComputeFullU);
        V_ = priorities_[prio].svd_.matrixV();
        S_.block(0,0,priorities_[prio].ny_,1) = priorities_[prio].svd_.singularValues();
        priorities_[prio].U_.block(0,0, priorities_[prio].ny_, priorities_[prio].ny_) = priorities_[prio].svd_.matrixU().transpose();
        cout<<"U: "<<priorities_[prio].U_<<endl;
        cout<<"V: "<<V_<<endl;
        cout<<"S: "<<S_<<endl;
        V_.resize(nx_, nx_);
        V_.setIdentity();
        priorities_[prio].U_.resize(priorities_[prio].ny_, nx_);
        S_.resize(nx_);*/

        cout<<priorities_[prio].A_proj_w_<<endl;

        int ret = KDL::svd_eigen_HH(priorities_[prio].A_proj_w_, priorities_[prio].U_, S_, V_, tmp_);
        if (ret < 0)
            throw std::runtime_error("Unable to perform svd");

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
        else{
            cout<<"TEST"<<endl;
            damping = sqrt(s_min*((1/norm_max_)-s_min));
        }

        cout<<"s_min: "<<s_min<<endl;
        cout<<"1/norm_max: "<<(1/norm_max_)/2<<endl;
        cout<<"Damping: "<<damping<<endl;
        cout<<"S: "<<endl;
        cout<<S_<<endl;

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        Damped_S_inv_.setZero();
        for (uint i = 0; i < min(nx_, priorities_[prio].ny_); i++)
            Damped_S_inv_(i,i) = (S_(i) / (S_(i) * S_(i) + damping * damping));

        cout<<"S_Inv: "<<endl;
        cout<<Damped_S_inv_<<endl;

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < S_.rows(); i++){
            if(S_(i) < epsilon_)
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / S_(i);
        }

        // A^# = Wq * V * S^# * U^T * Wy
        priorities_[prio].A_proj_inv_wls_ = joint_weight_mat_ * V_ * S_inv_ * priorities_[prio].U_.transpose() * priorities_[prio].task_weight_mat_; //Normal Inverse with weighting
        priorities_[prio].A_proj_inv_wdls_ = joint_weight_mat_ * V_ * Damped_S_inv_ * priorities_[prio].U_.transpose() * priorities_[prio].task_weight_mat_; //Damped inverse with weighting

        // x = x + A^# * y
        x += priorities_[prio].A_proj_inv_wdls_ * priorities_[prio].y_comp_;

        // Compute projection matrix for the next priority. Use here the undamped inverse to have a correct solution
        proj_mat_ -= priorities_[prio].A_proj_inv_wls_ * priorities_[prio].A_proj_;

    } //priority loop

    ///////////////
}


void HierarchicalWDLSSolver::setJointWeights(const Eigen::VectorXd& weights){
    if(weights.rows() == nx_){
        for(uint i = 0; i < nx_; i++){
            //In the original code, here, a choleski factorization + tranpose + inverse was done.
            //Since the weighting matrix is a diagonal matrix, this can be simplified to
            if(weights(i) == 0)
                joint_weight_mat_(i,i) = 1/sqrt(1e10);
            else
                joint_weight_mat_(i,i) = 1/sqrt(1/weights(i));
        }
        cout<<"Joint Weights: "<<endl;
        cout<<joint_weight_mat_<<endl;
    }
    else{
        LOG_ERROR("Cannot set joint weights. Size of weight vector is %i but number of joints is %i", weights.rows(), nx_);
        throw std::invalid_argument("Invalid Joint weight vector size");
    }
}

void HierarchicalWDLSSolver::setTaskWeights(const Eigen::VectorXd& weights, const uint prio){
    if(prio >= priorities_.size()){
        LOG_ERROR("Cannot set task weights. Given Priority is %i but number of priority levels is %i", prio, priorities_.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(priorities_[prio].ny_ != weights.rows()){
        LOG_ERROR("Cannot set task weights. No of task variables of priority %i is %i and number of weights is %i", prio, priorities_[prio].ny_, weights.size());
        throw std::invalid_argument("Invalid Weight vector size");
    }

    for(uint i = 0; i < priorities_[prio].ny_; i++){
        //In the original code, here, a choleski factorization + tranpose was done. Todo: Why?
        //Since the weighting matrix is a diagonal matrix, this can be simplified to
        priorities_[prio].task_weight_mat_(i,i) = sqrt(weights(i));
    }
}
