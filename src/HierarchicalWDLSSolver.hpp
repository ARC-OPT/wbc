#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include <vector>
#include <Eigen/SVD>


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
    class Priority{
    public:
        Priority(){}
        /**
         * @brief Priority Resizes members
         * @param ny Number of task variables of the current priority
         * @param nx Number of joint space variables
         */
        Priority(const unsigned int ny, const unsigned int nx){
            ny_ = ny;
            A_proj_.resize(ny, nx);
            A_proj_.setZero();
            A_proj_w_.resize(ny,nx);
            A_proj_w_.setZero();
            A_proj_inv_wls_.resize(ny, nx);
            A_proj_inv_wls_.setZero();
            A_proj_inv_wdls_.resize(ny, nx);
            A_proj_inv_wdls_.setZero();
            U_.resize(ny, nx);
            U_.setZero();
            A_proj_.setZero();
            y_comp_.resize(ny);
            task_weight_mat_.resize(ny, ny);
            task_weight_mat_.setIdentity();
            u_t_task_weight_mat_.resize(nx, ny);
            u_t_task_weight_mat_.setZero();
            svd_ = Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>(ny, nx);
            task_weight_mat_is_diagonal_ = true;
        }
        Eigen::MatrixXd A_proj_; /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_proj_w_; /** A Matrix projected on nullspace of the higher priority with weighting*/
        Eigen::MatrixXd A_proj_inv_wls_; /** Least square inverse of A_proj_w_*/
        Eigen::MatrixXd A_proj_inv_wdls_;  /** Damped Least square inverse of A_proj_w_*/
        Eigen::VectorXd y_comp_; /** Task variables which are compensated for the part of solution already met in higher priorities */
        Eigen::MatrixXd task_weight_mat_; /** Matrix containing information about the task weights*/


        unsigned int ny_; /** Number of task variables*/
        bool task_weight_mat_is_diagonal_;

        //Helpers
        Eigen::MatrixXd U_;
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_;
        Eigen::MatrixXd u_t_task_weight_mat_; /** Matrix U_transposed * task_weight_mat_s*/
    };

    HierarchicalWDLSSolver();
    ~HierarchicalWDLSSolver();

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
    void solve(const std::vector<Eigen::MatrixXd> &A,
                       const std::vector<Eigen::VectorXd> &y,
                       Eigen::VectorXd &x);

    /**
     * @brief setJointWeights Set full joint weight matrix.
     * @param weights Size must be no_joints * no_joints. Matrix must be symmetric and positive definite! This is not checked by the algorithm.
     *                If matrix is not diagonal, computation will increase a lot. If matrix is diagonal, a very HIGH entry means that
     *                the corresponding joint is not used at all for the solution. If matrix is diagonal, entries must be >= 0.
     */
    void setJointWeights(const Eigen::MatrixXd& weights);

    /**
     * @brief setTaskWeights Set full task weight matrix of the given priority. Note that, by using this method to set the task weights, the algorithm assumes that
     *                        the task weight matrix is not diagonal, which affects computation time a lot.
     * @param weights Size must be ny * ny. Matrix must be symmetric and positive definite! This is not checked by the algorithm.
     * @param prio Priority (>= 0) according to what was passed to configure() method
     */
    void setTaskWeights(const Eigen::MatrixXd& weights, const uint prio);

    double getCurDamping(){return damping_;}
    void setEpsilon(double epsilon){epsilon_ = epsilon;}
    void setNormMax(double norm_max){norm_max_ = norm_max;}
    Eigen::MatrixXd getJointWeights(){return joint_weight_mat_;}
    bool configured(){return configured_;}

protected:
    Eigen::MatrixXd proj_mat_; /** Projection Matrix */
    Eigen::MatrixXd S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    std::vector<Priority> priorities_; /** Priority vector */
    unsigned int nx_;
    double damping_;
    Eigen::MatrixXd joint_weight_mat_; /** Matrix containing joint weight information. */
    Eigen::MatrixXd Wq_V_, Wq_V_S_inv_, Wq_V_Damped_S_inv_;
    bool configured_;
    bool joint_weight_mat_is_diagonal_;

    //Properties
    double epsilon_; /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double norm_max_; /** Maximum norm of (J#) * y */

    //Helpers
    Eigen::VectorXd S_, tmp_;
    Eigen::MatrixXd V_, L_;

};
}
#endif

