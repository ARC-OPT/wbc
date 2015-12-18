#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include <vector>
#include <stdexcept>
#include "LinearEquationSystem.hpp"

namespace wbc{

/**
 * @brief Implementation of a hierarchical weighted damped least squares solver. This solver compute the solution for several hierarchially organized
 *        equation systems using nullspace projections. That is, the eqn system with the highest priority will be solved fully if n_rows <= n_cols
 *        the eqn system of the next priority will be solved as good as possible and so on. The solver may also include weights in solution (column) and input (row) space.
 */
class HierarchicalWDLSSolver{
public:

    /**
     * @brief The PriorityDataIntern class Manages all priority dependent information, i.e. all matrices that have to be resized according to
     * the number of rows per priority
     */
    class PriorityDataIntern{
    public:
        PriorityDataIntern(){}
        PriorityDataIntern(const unsigned int n_rows, const unsigned int n_cols){
            n_rows_ = n_rows;
            x_prio_.resize(n_cols);
            x_prio_.setZero();
            A_proj_.resize(n_rows, n_cols);
            A_proj_.setZero();
            A_proj_w_.resize(n_rows,n_cols);
            U_.resize(n_rows, n_cols);
            U_.setZero();
            A_proj_w_.setZero();
            A_proj_inv_wls_.resize(n_rows, n_cols);
            A_proj_inv_wls_.setZero();
            A_proj_inv_wdls_.resize(n_rows, n_cols);
            A_proj_inv_wdls_.setZero();
            y_comp_.resize(n_rows);
            y_comp_.setZero();
            row_weight_mat_.resize(n_rows, n_rows);
            row_weight_mat_.setIdentity();
            col_weight_mat_.resize(n_cols, n_cols);
            col_weight_mat_.setIdentity();
            u_t_row_weight_mat_.resize(n_cols, n_rows);
            u_t_row_weight_mat_.setZero();
            singular_values_.resize(n_cols);
        }
        Eigen::VectorXd x_prio_;
        Eigen::MatrixXd A_proj_;                /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_proj_w_;              /** A Matrix projected on nullspace of the higher priority with weighting*/
        Eigen::MatrixXd U_;                     /** Matrix of left singular vector of A_proj_w_ */
        Eigen::MatrixXd A_proj_inv_wls_;        /** Least square inverse of A_proj_w_*/
        Eigen::MatrixXd A_proj_inv_wdls_;       /** Damped Least square inverse of A_proj_w_*/
        Eigen::VectorXd y_comp_;                /** Input variables which are compensated for the part of solution already met in higher priorities */
        Eigen::MatrixXd row_weight_mat_;       /** Row (input) weight matrix. Has to be positive definite*/
        Eigen::MatrixXd col_weight_mat_;       /** Row (input) weight matrix. Has to be positive definite*/
        Eigen::MatrixXd u_t_row_weight_mat_;   /** Matrix U_transposed * row_weight_mat_*/
        Eigen::VectorXd singular_values_;       /** Singular values of this priprity */
        double damping_;                         /** Damping factor for matrix inversion on this priority */

        unsigned int n_rows_;                   /** Number of input variables of this priority*/
    };

    HierarchicalWDLSSolver();
    ~HierarchicalWDLSSolver(){}

    /**
     * @brief configure Resizes member variables
     * @param n_rows_per_prio Number of rows per priority
     * @param nx Number of solution space variables (number of columns)
     * @return True in case of successful initialization, false else
     */
    bool configure(const std::vector<int>& n_rows_per_prio, const unsigned int nx);

    /**
     * @brief solve Compute optimal control solution
     * @param linear_eqn_pp A Linear equation system, sorted by priority
     * @param x solution
     */
    void solve(const std::vector<LinearEquationSystem> &linear_eqn_pp,
               Eigen::VectorXd &x);


    void setColumnWeights(const Eigen::VectorXd& weights, const uint prio);
    void setRowWeights(const Eigen::VectorXd& weights, const uint prio);
    void setEpsilon(double epsilon);
    void setNormMax(double norm_max);
    bool configured(){return configured_;}
    std::vector<int> getNyPerPriority(){return n_rows_per_prio_;}
    uint getNoPriorities(){return n_rows_per_prio_.size();}
    const PriorityDataIntern& getPriorityData(const uint prio);



protected:
    std::vector<PriorityDataIntern> priorities_;  /** Contains priority specific matrices etc. */
    std::vector<int> n_rows_per_prio_;
    Eigen::MatrixXd proj_mat_;          /** Projection Matrix */
    Eigen::VectorXd S_;                 /** Eigenvalue vector*/
    Eigen::MatrixXd V_;                 /** Matrix of right singular vectors*/
    Eigen::MatrixXd S_inv_;             /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_;      /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    Eigen::MatrixXd Wq_V_;              /** Column weight mat times Matrix of Vectors of right singular vectors*/
    Eigen::MatrixXd Wq_V_S_inv_;        /** Wq_V_ times S_inv_ */
    Eigen::MatrixXd Wq_V_Damped_S_inv_; /** Wq_V_ times Damped_S_inv_ */

    unsigned int n_cols_;                   /** No of columns */
    bool configured_;                   /** Has configure been called yet?*/

    //Properties
    double epsilon_;    /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double norm_max_;   /** Maximum norm of (J#) * y */

    //Helpers
    Eigen::VectorXd tmp_;
};
}
#endif

