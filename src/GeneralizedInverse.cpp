#include "GeneralizedInverse.hpp"
#include <stdexcept>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <algorithm>
#include <base/Time.hpp>


namespace wbc{

GeneralizedInverse::GeneralizedInverse(const uint n_rows, const uint n_cols)
{
    n_rows_ = n_rows;
    n_cols_ = n_cols;

    //Init weights with ones
    row_weights_.setOnes(n_rows);
    col_weights_.setOnes(n_cols);

    sqrt_row_weights_.setOnes(n_rows);
    sqrt_col_weights_.setOnes(n_cols);

    weighted_mat_.resize(n_rows, n_cols);
    weighted_mat_.setZero();

    singular_vals_.resize(n_cols);
    singular_vals_.setZero();

    S_inv_.resize(n_cols, n_cols);
    S_inv_.setZero();

    V_.resize(n_cols, n_cols);
    V_.setZero();

    U_.resize(n_rows, n_cols);
    U_.setZero();

    cw_V_.resize(n_cols, n_cols);
    cw_V_.setZero();

    cw_V_S_inv_.resize(n_cols, n_cols);
    cw_V_S_inv_.setZero();

    U_transp_rw_.resize(n_cols, n_rows);

    tmp_.resize(n_cols);
    tmp_.setZero();

    svd_ = Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>(n_rows, n_cols);

    damping_ = 0;
    damping_method_ = constant_damping;
    svd_method_ = svd_kdl;
    epsilon_ = 1e-9;
}

void GeneralizedInverse::computeInverse(const Eigen::MatrixXd &in, Eigen::MatrixXd &out)
{
    // Validate input and output matrix

    base::Time begin = base::Time::now();
    base::Time start = base::Time::now();

    if( in.rows() != n_rows_ ||
            in.cols() != n_cols_){
        error_stream_ << "GeneralizedInverse::computeInverse: Problem size is " << n_rows_ << "x" << n_cols_ << " input matrix has size " << in.rows() << "x" << in.cols();
        throw std::invalid_argument(error_stream_.str());
    }

    if( out.rows() != n_cols_ ||
         out.cols() != n_rows_){
         error_stream_ << "GeneralizedInverse::computeInverse: Problem size is " << n_rows_ << "x" << n_cols_ << " output matrix has size " << out.rows() << "x" << out.cols();
         throw std::invalid_argument(error_stream_.str());
     }

    // Multiply row weights:

    for(uint i = 0; i < n_rows_; i++)
        weighted_mat_.row(i) = sqrt_row_weights_(i) * in.row(i);


    // Multiply column weights:

    for(uint i = 0; i < n_cols_; i++)
        weighted_mat_.col(i) = sqrt_col_weights_(i) * weighted_mat_.col(i);

    time_weighting_ = (base::Time::now() - start).toSeconds();
    start = base::Time::now();


    // Compute SVD:

    if(svd_method_ == svd_eigen)
    {
        //Compute svd of A Matrix: A = U*Sigma*V^T, where Sigma contains the singular values on its main diagonal
        svd_.compute(weighted_mat_, Eigen::ComputeFullV | Eigen::ComputeFullU);

        V_ = svd_.matrixV();
        uint ns = svd_.singularValues().size(); //No of singular values

        //Entries of S that are not singular values will be zero
        singular_vals_.setZero();
        singular_vals_.block(0,0,ns,1) = svd_.singularValues();

        //U output of svd will have different size than required, copy only the first ns columns. They contain the relevant Eigenvectors
        U_.block(0,0, n_rows_, ns) = svd_.matrixU().block(0,0,n_rows_, ns);
    }
    else if(svd_method_ == svd_kdl)
    {
        KDL::svd_eigen_HH(weighted_mat_, U_, singular_vals_, V_, tmp_);
    }
    else{
        error_stream_ << "Invalid SVD method: " << svd_method_;
        throw std::invalid_argument(error_stream_.str());
    }

    // Compute Damping factor, if not constant:

    if(damping_method_ == variable_damping)
    {
        double s_min = singular_vals_.block(0,0,std::min(n_cols_, n_rows_),1).minCoeff();
        if(s_min <= (1/norm_max_)/2)
            damping_ = (1/norm_max_)/2;
        else if(s_min >= (1/norm_max_))
            damping_ = 0;
        else
            damping_ = sqrt(s_min*((1/norm_max_)-s_min));
    }

    // Invert Singular values:

    S_inv_.setZero();
    if( damping_ == 0)
    {
        for(uint i = 0; i < singular_vals_.rows(); i++){
            if(singular_vals_(i) < epsilon_)
                S_inv_(i,i) = 0;
            else
                S_inv_(i,i) = 1 / singular_vals_(i);
        }
    }
    else
    {
        for (uint i = 0; i < std::min(n_cols_, n_rows_); i++)
            S_inv_(i,i) = (singular_vals_(i) / (singular_vals_(i) * singular_vals_(i) + damping_ * damping_));
    }

    time_svd_ = (base::Time::now() - start).toSeconds();
    start = base::Time::now();

    // Compute weighted generalized inverse: A^# = Wc^-1 * V * S^# * U^T * Wr

    U_transp_rw_ = U_.transpose();
    for(uint i = 0; i < n_rows_; i++)
        U_transp_rw_.col(i) = U_transp_rw_.col(i) * sqrt_row_weights_(i);

    for(uint i = 0; i < n_cols_; i++)
        cw_V_.row(i) = sqrt_col_weights_(i) * V_.row(i);

    for(uint i = 0; i < n_cols_; i++)
        cw_V_S_inv_.col(i) = cw_V_.col(i) * S_inv_(i,i);

    out = cw_V_S_inv_ * U_transp_rw_;

    time_multiplying_ = (base::Time::now() - start).toSeconds();
    time_total_ = (base::Time::now() - begin).toSeconds();

}

void GeneralizedInverse::setRowWeights(const Eigen::VectorXd& row_weights)
{
    if(row_weights.size() != n_rows_){
        error_stream_ << "GeneralizedInverse::setRowWeights: Number of cols is " << n_rows_ << " but number of column weights given is " << row_weights.size();
        throw std::invalid_argument(error_stream_.str());
    }
    row_weights_ = row_weights;
    for(uint i = 0; i < n_rows_; i++)
        sqrt_row_weights_(i) = sqrt(row_weights(i));
}

void GeneralizedInverse::setColWeights(const Eigen::VectorXd& col_weights)
{
    if(col_weights.size() != n_cols_){
        error_stream_ << "GeneralizedInverse::setColWeights: Number of cols is " << n_cols_ << " but number of column weights given is " << col_weights.size();
        throw std::invalid_argument(error_stream_.str());
    }
    col_weights_ = col_weights;
    for(uint i = 0; i < n_cols_; i++)
        sqrt_col_weights_(i) = sqrt(col_weights(i));
}

void GeneralizedInverse::setSVDMethod(const svd_method method)
{
    svd_method_ = method;
}

void GeneralizedInverse::setNormMaxDamping(const double norm)
{
    if(norm <= 0){
        error_stream_ << "GeneralizedInverse::setNormMaxDamping: Maximum Norm has to be > 0, but is " << norm;
        throw std::invalid_argument(error_stream_.str());
    }
    norm_max_ = norm;
    damping_method_ = variable_damping;
}

void GeneralizedInverse::setConstantDamping(const double damping)
{
    if(damping <= 0){
        error_stream_ << "GeneralizedInverse::setConstantDamping: Damping has to be > 0, but is " << damping;
        throw std::invalid_argument(error_stream_.str());
    }
    damping_method_ = constant_damping;
    damping_ = damping;
}

void GeneralizedInverse::setSingularValueEpsilon(const double epsilon)
{
    if(epsilon <= 0){
        error_stream_ << "GeneralizedInverse::setSingularValueEpsilon: Epsilon has to be > 0, but is " << epsilon;
        throw std::invalid_argument(error_stream_.str());
    }
    epsilon_ = epsilon;
}

}
