#ifndef GENERALIZEDINVERSE_HPP
#define GENERALIZEDINVERSE_HPP

#include <Eigen/Core>
#include <Eigen/SVD>

namespace wbc{

enum svd_method{svd_eigen, svd_kdl};
enum damping_method{constant_damping, variable_damping};

class GeneralizedInverse{
public:
    GeneralizedInverse(const uint n_rows, const uint n_cols);

    /** Perform inversion. Both, input matrix has to have size n_rows x n_cols, output matrix size n_cols x n_rows */
    void computeInverse(const Eigen::MatrixXd& in, Eigen::MatrixXd& out);

    /** Set row weights. Has to be same size as n_rows */
    void setRowWeights(const Eigen::VectorXd& row_weights);

    /** Set column weights. Has to be same size as n_cols */
    void setColWeights(const Eigen::VectorXd& col_weights);

    /** Set SVD method to used. Can be either svd_eigen or svd_kdl. The latter seems to be faster for large matrices */
    bool setSVDMethod(const svd_method method);

    /** Set Maximum norm for automatic damping computation. A low value will provide a strongly damped solution near singularities,
     *  a very high value will provide the undamped solution. However, this should better be achived by setting a constant damping of zero */
    void setNormMaxDamping(const double norm);

    /** Set a constant damping factor for computing the inverse. */
    void setConstantDamping(const double damping);

    /** Set epsilon for inversion of Singular values to avoid numerical problems at configuration with zero damping */
    void setSingularValueEpsilon(const double epsilon);

    uint n_rows_; /** No of matrix rows */
    uint n_cols_; /** No of matrix columns */

    damping_method damping_method_; /** Current damping method */
    svd_method svd_method_;         /** Current SVD method */

    double damping_;   /** Current damping factor */
    double norm_max_;  /** Maximum norm of the LS solution of the equation system defined by the input matrix. Will be used to automatically compute a suitable damping
                           close to singular configurations */
    double epsilon_;   /** Parameter to preserve numerical stability when inverting the Singular values without damping */

    Eigen::VectorXd row_weights_; /** Row weight vector */
    Eigen::VectorXd col_weights_; /** Column weight vector */
    Eigen::VectorXd sqrt_row_weights_; /** Sqrt of Row weight vector */
    Eigen::VectorXd sqrt_col_weights_; /** Sqrt of Column weight vector */

    Eigen::MatrixXd weighted_mat_;  /** Input matrix with weighting */
    Eigen::VectorXd singular_vals_; /** Vector of singular values of weighted_mat_. SVD is given as mat = U * S * V_T */
    Eigen::MatrixXd S_inv_;         /** Inverted singular values */
    Eigen::MatrixXd U_transp_rw_;   /** U_T * row_weights */
    Eigen::MatrixXd cw_V_;          /** Column weights * V */
    Eigen::MatrixXd cw_V_S_inv_;    /** Column weights * V * S_inv_ */

    //Helpers for svd
    Eigen::MatrixXd U_, V_;
    Eigen::VectorXd tmp_;
    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_; /** For singular value decomposition used in matrix inversion*/

    //Performance parameters:
    double time_weighting_;
    double time_svd_;
    double time_multiplying_;
    double time_total_;

    std::stringstream error_stream_;

};
}
#endif // GENERALIZEDINVERSE_HPP
