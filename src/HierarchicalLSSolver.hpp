#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include "HierarchicalSolver.hpp"
#include <Eigen/SVD>

class HierarchicalLSSolver : HierarchicalSolver{

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
            task_weight_mat_.setZero();
            svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(ny, nx);
        }
        Eigen::MatrixXd A_proj_; /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_proj_w_; /** A Matrix projected on nullspace of the higher priority with weighting*/
        Eigen::MatrixXd A_proj_inv_wls_; /** Least square inverse of A_proj_w_*/
        Eigen::MatrixXd A_proj_inv_wdls_;  /** Damped Least square inverse of A_proj_w_*/
        Eigen::VectorXd y_comp_; /** Task variables which are compensated for the part of solution already met in higher priorities */
        Eigen::MatrixXd task_weight_mat_; /** Matrix containing information about the task weights*/

        unsigned int ny_; /** Number of task variables*/


        //Helpers
        Eigen::MatrixXd U_;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_;

    };

    HierarchicalLSSolver();

    /**
     * @brief configure Resizes member variables
     * @param ny_per_prio Number of task variables per priority
     * @param nx Number of joint space variables
     * @return True in case of successful initialization, false else
     */
    virtual bool configure(const std::vector<unsigned int>& ny_per_prio, const unsigned int nx);

    /**
     * @brief solve Compute optimal control solution
     * @param A Linear equation system describing the tasks, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param y Task variables, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param x Control solution in joint space
     */
    virtual void solve(const std::vector<Eigen::MatrixXd> &A,
                       const std::vector<Eigen::VectorXd> &y,
                       Eigen::VectorXd &x);

    void setEpsilon(double epsilon){epsilon_ = epsilon;}
    void setNormMax(double norm_max){norm_max_ = norm_max;}
    void setJointWeights(const Eigen::VectorXd& weights);
    void setTaskWeights(const Eigen::VectorXd& weights, const uint prio);

protected:
    Eigen::MatrixXd proj_mat_; /** Projection Matrix */
    Eigen::MatrixXd S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    std::vector<Priority> priorities_; /** Priority vector */
    unsigned int nx_;
    Eigen::MatrixXd joint_weight_mat_; /** Matrix containing joint weight information. */

    //Properties
    double norm_max_; /** Maximum norm of (J#) * y */
    double epsilon_; /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/

    //Helpers
    Eigen::VectorXd S_, tmp_;
    Eigen::MatrixXd V_;

};

#endif
