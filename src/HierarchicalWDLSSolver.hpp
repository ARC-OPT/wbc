#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include "HierarchicalSolver.hpp"

class HierarchicalWDLSSolver : HierarchicalSolver{

public:
    class Priority{
    public:
        Priority(){}
        Priority(const unsigned int no_task_vars, const unsigned int no_of_joints){
            no_task_vars_ = no_task_vars;
            A_projected_.resize(no_task_vars, no_of_joints);
            A_projected_.setZero();
            A_projected_inv_.resize(no_task_vars, no_of_joints);
            A_projected_inv_.setZero();
            U_.resize(no_task_vars, no_of_joints);
            U_.setZero();
            A_projected_.setZero();
        }
        Eigen::MatrixXd A_projected_; /** A Matrix projected on nullspace of the higher priority*/
        Eigen::MatrixXd A_projected_inv_; /** A Matrix projected on nullspace of the higher priority (Inverted)*/

        //Helpers
        Eigen::MatrixXd U_;

        unsigned int no_task_vars_;
    };

    HierarchicalWDLSSolver();

    /**
     * @brief configure Initialize member variables. Further Configurations will have to be done by the derived class, since they will propably be solver specific
     * @param ny_per_priority No of task variables per priority level.
     * @param nx No of independent configuration space variables
     * @return False in case of an error, true in case of success
     */
    virtual bool configure(const std::vector<unsigned int>& no_of_constr_per_priority, const unsigned int no_of_joints);

    /**
     * @brief solve
     * @param A
     * @param y
     * @param solver_output
     * @return
     */
    virtual void solve(const std::vector<Eigen::MatrixXd> &A_prio,
                       const std::vector<Eigen::VectorXd> &y_prio,
                       Eigen::VectorXd &solver_output);

    void setEpsilon(double epsilon){epsilon_ = epsilon;}
    void setNormMax(double norm_max){norm_max_ = norm_max;}

protected:
    Eigen::MatrixXd proj_mat_; /** Projection Matrix */
    Eigen::MatrixXd prev_proj_mat_; /** projection matrix of previous priority */
    Eigen::MatrixXd S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix*/
    Eigen::MatrixXd Damped_S_inv_; /** Diagonal matrix containing the reciprocal eigenvalues of the A matrix with damping*/
    std::vector<Priority> priorities_; /** Priority vector */
    unsigned int no_of_joints_;

    //Properties
    double norm_max_; /** Maximum norm of the (J#)Y */
    double epsilon_; /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/

    //Helpers
    Eigen::VectorXd S_, tmp_;
    Eigen::MatrixXd V_;
};

#endif
