#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include "SolverTypes.hpp"
#include <vector>
#include <Eigen/SVD>
#include "PriorityData.hpp"

namespace wbc{


/**
 * @brief Implementation of a hierarchical weighted damped least squares solver. This solver may cope with several hierarchially organized
 *        tasks, which will be solved using nullspace projections. That is, the task with the highest priority will be solved fully if m <= n
 *        (where m is the number of task variables and n the number of independent joint variables), the task of the next priority will be
 *        solved as good as possible (depending on the degree of redundancy) and so on.
 *        The solver may also include weights in joint and task space.
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
         * @param nx Number of joint space variables
         */
        PriorityDataIntern(const unsigned int ny, const unsigned int nx){
            ny_ = ny;
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
            task_weight_mat_.resize(ny, ny);
            task_weight_mat_.setIdentity();
            u_t_task_weight_mat_.resize(nx, ny);
            u_t_task_weight_mat_.setZero();
            task_weight_mat_is_diagonal_ = true;
            svd_ = Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>(ny, nx);
        }
        Eigen::MatrixXd A_proj_;                /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_proj_w_;              /** A Matrix projected on nullspace of the higher priority with weighting*/
        Eigen::MatrixXd U_;                     /** Matrix of left singular vector of A_proj_w_ */
        Eigen::MatrixXd A_proj_inv_wls_;        /** Least square inverse of A_proj_w_*/
        Eigen::MatrixXd A_proj_inv_wdls_;       /** Damped Least square inverse of A_proj_w_*/
        Eigen::VectorXd y_comp_;                /** Task variables which are compensated for the part of solution already met in higher priorities */
        Eigen::MatrixXd task_weight_mat_;       /** Task weight matrix. Has to be positive definite*/
        Eigen::MatrixXd u_t_task_weight_mat_;   /** Matrix U_transposed * task_weight_mat_*/

        unsigned int ny_;                   /** Number of task variables*/
        bool task_weight_mat_is_diagonal_;  /**  Is the task weight mat a diagonal matrix. This is important regarding the computation time*/

        //Helpers
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_; /** For singular value decomposition used in matrix inversion*/
    };

    HierarchicalWDLSSolver();
    ~HierarchicalWDLSSolver(){}

    /**
     * @brief configure Resizes member variables
     * @param ny_per_prio Number of task variables per priority
     * @param nx Number of joint space variables
     * @return True in case of successful initialization, false else
     */
    bool configure(const std::vector<uint>& ny_per_prio, const unsigned int nx);

    /**
     * @brief solve Compute optimal control solution
     * @param A Linear equation system describing the tasks, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param y Task variables, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param x Control solution in joint space
     */
    void solve(const SolverInput& input, Eigen::VectorXd &x);


    /**
     * @brief setJointWeights Set full joint weight matrix.
     * @param weights Size must be no_joints * no_joints. Matrix must be symmetric and positive definite! This is not checked by the algorithm.
     *                If matrix is not diagonal, computation will increase a lot. If matrix is diagonal, a very HIGH entry means that
     *                the corresponding joint is not used at all for the solution. If matrix is diagonal, entries must be >= 0.
     */
    void setJointWeights(const Eigen::VectorXd& weights);

    /**
     * @brief setTaskWeights Set full task weight matrix of the given priority. Note that, by using this method to set the task weights, the algorithm assumes that
     *                        the task weight matrix is not diagonal, which affects computation time a lot.
     * @param weights Size must be ny * ny. Matrix must be symmetric and positive definite! This is not checked by the algorithm.
     * @param prio Priority (>= 0) according to what was passed to configure() method
     */
    void setTaskWeights(const Eigen::VectorXd& weights, const uint prio);

    void setEpsilon(double epsilon){epsilon_ = epsilon;}
    void setNormMax(double norm_max){norm_max_ = norm_max;}
    void getJointWeights(Eigen::VectorXd &weights){weights = joint_weight_mat_.diagonal();}
    bool configured(){return configured_;}
    void setSVDMethod(svd_method method){svd_method_ = method;}
    void setComputeDebug(const bool compute_debug){compute_debug_ = compute_debug;}
    void getPrioDebugData(std::vector<PriorityData>& data){data = prio_debug_;}
    std::vector<uint> getNyPerPriority(){return ny_per_prio_;}
    uint getNoPriorities(){return ny_per_prio_.size();}

    std::vector<PriorityDataIntern> priorities_;  /** Contains priority specific matrices etc. */
    std::vector<PriorityData> prio_debug_;      /** Contains debug information for each priority */
    std::vector<uint> ny_per_prio_;

protected:
    Eigen::MatrixXd proj_mat_;          /** Projection Matrix */
    Eigen::VectorXd S_;                 /** Eigenvalue vector of the task matrices */
    Eigen::MatrixXd V_;                 /** Matrix of right singular vectors*/
    Eigen::MatrixXd S_inv_;             /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_;      /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    Eigen::MatrixXd joint_weight_mat_;  /** Joint weight matrix. Has to be positive definite */
    Eigen::MatrixXd Wq_V_;              /** Joint weight mat times Matrix of Vectors of right singular vectors*/
    Eigen::MatrixXd Wq_V_S_inv_;        /** Wq_V_ times S_inv_ */
    Eigen::MatrixXd Wq_V_Damped_S_inv_; /** Wq_V_ times Damped_S_inv_ */
    Eigen::MatrixXd L_;                 /** Choleski Decomposition: If the joint weight matrix is not diagonal, it has to be decomposed into lower triangular matrix and its transpose using method of Cholesky*/

    unsigned int nx_;                   /** No of robot joints */
    bool configured_;                   /** Has configure been called yet?*/
    bool joint_weight_mat_is_diagonal_; /** Is the joint weight mat a diagonal matrix. This is important regarding the computation time*/
    bool compute_debug_;                /** Compute additional debug info */

    //Properties
    double epsilon_;    /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double norm_max_;   /** Maximum norm of (J#) * y */
    svd_method svd_method_;

    //Helpers
    Eigen::VectorXd tmp_;
};
}
#endif

