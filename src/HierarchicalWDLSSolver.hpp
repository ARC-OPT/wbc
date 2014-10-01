#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include <vector>
#include <Eigen/SVD>
#include "GeneralizedInverse.hpp"

namespace wbc{


/**
 * @brief Implementation of a hierarchical weighted damped least squares solver. This solver may cope with several hierarchially organized
 *        tasks, which will be solved using nullspace projections. That is, the task with the highest priority will be solved fully if m <= n
 *        (where m is the number of task variables and n the number of independent solution variables), the task of the next priority will be
 *        solved as good as possible (depending on the degree of redundancy) and so on.
 *        The solver may also include weights in solution and input space.
 */
class HierarchicalWDLSSolver{
public:

    /**
     * @brief The PriorityDataIntern class Manages all priority dependent information, i.e. all matrices that have to be resized according to
     * the number of tasks per priority
     */
    class PriorityDataIntern{
    public:
        PriorityDataIntern(){}
        /**
         * @brief Priority Resizes members
         * @param ny Number of task variables of the current priority
         * @param nx Number of solution space variables
         */
        PriorityDataIntern(const unsigned int ny, const unsigned int nx){
            ny_ = ny;
            x_prio_.resize(nx);
            x_prio_.setZero();
            A_proj_.resize(ny, nx);
            A_proj_.setZero();
            A_proj_w_.resize(ny,nx);
            U_.resize(ny, nx);
            U_.setZero();
            A_proj_w_.setZero();
            A_proj_inv_wls_.resize(ny, nx);
            A_proj_inv_wls_.setZero();
            A_proj_inv_wdls_.resize(ny, nx);
            A_proj_inv_wdls_.setZero();
            y_comp_.resize(ny);
            y_comp_.setZero();
            row_weight_mat_.resize(ny, ny);
            row_weight_mat_.setIdentity();
            u_t_row_weight_mat_.resize(nx, ny);
            u_t_row_weight_mat_.setZero();
            svd_ = Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>(ny, nx);
            singular_values_.resize(ny);
        }
        Eigen::VectorXd x_prio_;
        Eigen::MatrixXd A_proj_;                /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_proj_w_;              /** A Matrix projected on nullspace of the higher priority with weighting*/
        Eigen::MatrixXd U_;                     /** Matrix of left singular vector of A_proj_w_ */
        Eigen::MatrixXd A_proj_inv_wls_;        /** Least square inverse of A_proj_w_*/
        Eigen::MatrixXd A_proj_inv_wdls_;       /** Damped Least square inverse of A_proj_w_*/
        Eigen::VectorXd y_comp_;                /** Task variables which are compensated for the part of solution already met in higher priorities */
        Eigen::MatrixXd row_weight_mat_;       /** Task weight matrix. Has to be positive definite*/
        Eigen::MatrixXd u_t_row_weight_mat_;   /** Matrix U_transposed * row_weight_mat_*/
        Eigen::VectorXd singular_values_;       /** Singular values of this priprity */
        double damping_;                         /** Damping factor for matrix inversion on this priority */

        unsigned int ny_;                   /** Number of task variables*/

        //Helpers
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_; /** For singular value decomposition used in matrix inversion*/
    };

    HierarchicalWDLSSolver();
    ~HierarchicalWDLSSolver(){}

    /**
     * @brief configure Resizes member variables
     * @param ny_per_prio Number of task variables per priority
     * @param nx Number of solution space variables
     * @return True in case of successful initialization, false else
     */
    bool configure(const std::vector<int>& ny_per_prio, const unsigned int nx);

    /**
     * @brief solve Compute optimal control solution
     * @param A Linear equation system describing the tasks, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param y Task variables, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param x Control solution
     */
    void solve(const std::vector<Eigen::MatrixXd> &A,
               const std::vector<Eigen::VectorXd> &Wy,
               const std::vector<Eigen::VectorXd> &y_ref,
               const Eigen::VectorXd &Wx,
               Eigen::VectorXd &x);


    void setColumnWeights(const Eigen::VectorXd& weights);
    void setRowWeights(const Eigen::VectorXd& weights, const uint prio);
    void setEpsilon(double epsilon){epsilon_ = epsilon;}
    void setNormMax(double norm_max){norm_max_ = norm_max;}
    void getColumnWeights(Eigen::VectorXd &weights){weights = column_weights_;}
    bool configured(){return configured_;}
    void setSVDMethod(svd_method method){svd_method_ = method;}
    std::vector<int> getNyPerPriority(){return ny_per_prio_;}
    uint getNoPriorities(){return ny_per_prio_.size();}
    const PriorityDataIntern& getPriorityData(const uint prio);



protected:
    std::vector<PriorityDataIntern> priorities_;  /** Contains priority specific matrices etc. */
    std::vector<int> ny_per_prio_;
    Eigen::MatrixXd proj_mat_;          /** Projection Matrix */
    Eigen::VectorXd S_;                 /** Eigenvalue vector of the task matrices */
    Eigen::MatrixXd V_;                 /** Matrix of right singular vectors*/
    Eigen::MatrixXd S_inv_;             /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_;      /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    Eigen::MatrixXd column_weight_mat_;  /** Column weight matrix. Has to be positive definite */
    Eigen::MatrixXd Wq_V_;              /** Column weight mat times Matrix of Vectors of right singular vectors*/
    Eigen::MatrixXd Wq_V_S_inv_;        /** Wq_V_ times S_inv_ */
    Eigen::MatrixXd Wq_V_Damped_S_inv_; /** Wq_V_ times Damped_S_inv_ */

    unsigned int nx_;                   /** No of columns */
    bool configured_;                   /** Has configure been called yet?*/

    //Properties
    double epsilon_;    /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double norm_max_;   /** Maximum norm of (J#) * y */
    svd_method svd_method_;
    Eigen::VectorXd column_weights_;

    //Helpers
    Eigen::VectorXd tmp_;
};
}
#endif

