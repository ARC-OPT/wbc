#ifndef HIERARCHICAL_WDLS_SOLVER_HPP
#define HIERARCHICAL_WDLS_SOLVER_HPP

#include "Solver.hpp"

namespace wbc{

class OptProblem;

/**
 * @brief Implementation of a hierarchical weighted damped least squares solver. This solver compute the solution for several hierarchially organized
 *        equation systems using nullspace projections. That is, the eqn system with the highest priority will be solved fully if n_rows <= n_cols
 *        the eqn system of the next priority will be solved as good as possible and so on. The solver may also include weights in solution (column) and input (row) space.
 */
class HierarchicalLeastSquaresSolver : public Solver{
public:

    /**
     * @brief The PriorityDataIntern class Manages all priority dependent information, i.e. all matrices that have to be resized according to
     * the number of rows per priority
     */
    class PriorityData{
    public:
        PriorityData(){}
        PriorityData(const unsigned int _n_constraint_variables, const unsigned int n_joints){
            n_constraint_variables = _n_constraint_variables;
            solution_prio.setZero(n_joints);
            A_proj.setZero(_n_constraint_variables, n_joints);
            A_proj_w.setZero(_n_constraint_variables,n_joints);
            U.setZero(_n_constraint_variables, n_joints);
            A_proj_inv_wls.setZero(_n_constraint_variables, n_joints);
            A_proj_inv_wdls.setZero(_n_constraint_variables, n_joints);
            y_comp.setZero(_n_constraint_variables);
            constraint_weight_mat.resize(_n_constraint_variables, _n_constraint_variables);
            constraint_weight_mat.setIdentity();
            joint_weight_mat.resize(n_joints, n_joints);
            joint_weight_mat.setIdentity();
            u_t_weight_mat.setZero(n_joints, _n_constraint_variables);
            sing_vals.resize(n_joints);
        }
        base::VectorXd solution_prio;         /** Solution for the current priority*/
        base::MatrixXd A_proj;                /** Constraint Matrix projected into nullspace of the higher priority */
        base::MatrixXd A_proj_w;              /** Constraint Matrix projected into nullspace of the higher priority with weighting*/
        base::MatrixXd U;                     /** Matrix of left singular vector of A_proj_w */
        base::MatrixXd A_proj_inv_wls;        /** Least square inverse of A_proj_w*/
        base::MatrixXd A_proj_inv_wdls;       /** Damped Least square inverse of A_proj_w*/
        base::VectorXd y_comp;                /** Input variables which are compensated for the part of solution already met in higher priorities */
        base::MatrixXd constraint_weight_mat; /** Constraint weight matrix of this priority*/
        base::MatrixXd joint_weight_mat;      /** Joint weight matrix of this priority*/
        base::MatrixXd u_t_weight_mat;        /** Matrix U_transposed * constraint_weight_mat*/
        base::VectorXd sing_vals;             /** Singular values of this priority */
        double damping;                        /** Damping term for matrix inversion on this priority*/
        unsigned int n_constraint_variables;   /** Number of constraint variables of this priority*/
    };

    HierarchicalLeastSquaresSolver();
    ~HierarchicalLeastSquaresSolver(){}

    /**
     * @brief configure Resizes member variables
     * @param constraint_variables_per_prio Number of constraint variables per priority, i.e. number of row of the constraint Jacobian of that priority
     * @param no_of_joints Number of robot joints
     * @return true in case of successful initialization, false otherwise
     */
    bool configure(const std::vector<int>& n_constraint_per_prio, const unsigned int n_joints);

    /**
     * @brief solve Compute optimal control solution
     * @param linear_eqn_pp A Linear equation system, sorted by priority
     * @param x solution
     */
    virtual void solve(const OptProblem &opt_problem, base::VectorXd &solver_output);

    /**
     * @brief setJointWeights Sets the joint weight vector for the given priority
     * @param weights Has to have same size as number of joints. Values have to be >= 0! A values of 0 means that the joint does not
     *                contribute to the solution at all.
     * @param prio Priority the joint weight vector should be applied to
     */
    void setJointWeights(const base::VectorXd& weights, const uint prio);

    /**
     * @brief setConstraintWeights Sets the weights for the constraints of the given priority.
     * @param weights Size has to be same number of constraint variables of that priority.
     * @param prio Priority the constraint weight vector should be applied to
     */
    void setConstraintWeights(const base::VectorXd& weights, const uint prio);

    /**
     * @brief setMinEigenvalue Sets the minimum Eigenvalue that is allowed to occur in normal (undamped) matrix inversion.
     *        This matrix inversion is used for nullspace projection. An Eigenvalue is set to zero if it smaller that
     *        min_eigenvalue for sake of numerical stability.
     * @param min_eigenvalue Has to be > 0
     */
    void setMinEigenvalue(double min_eigenvalue);

    /**
     * @brief setMaxSolverOutputNorm Sets the maximum norm term. The solution of the solver will have a norm that is below this value.
     *        This value will be used to compute a suitable damping factor for matrix inversion.
     * @param norm_max Maximum output norm. Has to be > 0!
     */
    void setMaxSolverOutputNorm(double norm_max);

    /**
     * @brief setMaxSolverOutput Set max. values for each entry of the solver output vector.
     *        If one value exceeds this maximum, all other values will be scaled accordingly.
     * @param max_solver_output Entries have to be > 0! Size has to be same as number of joints.
     */
    void setMaxSolverOutput(const base::VectorXd& max_solver_output);

    /**
     * @brief applySaturation Apply maximum solver output.
     */
    void applySaturation(base::VectorXd& solver_output);

protected:
    std::vector<PriorityData> priorities;     /** Contains priority specific matrices etc. */
    base::MatrixXd proj_mat;                 /** Projection Matrix that performs the nullspace projection onto the next lower priority*/
    base::VectorXd s_vals;                   /** Singular value vector*/
    base::MatrixXd s_vals_inv;               /** Diagonal matrix containing the reciprocal singular values*/
    base::MatrixXd sing_vect_r;              /** Matrix of right singular vectors*/
    base::MatrixXd damped_s_vals_inv;        /** Diagonal matrix containing the reciprocal singular values with damping*/
    base::MatrixXd Wq_V;                     /** Column weight matrix times Matrix of Vectors of right singular vectors*/
    base::MatrixXd Wq_V_s_vals_inv;          /** Wq_V * s_vals_inv */
    base::MatrixXd Wq_V_damped_s_vals_inv;   /** Wq_V * damped_s_vals_inv */

    unsigned int no_of_joints;             /** Number of joints */
    bool configured;                       /** Has configure been called yet?*/
    base::VectorXd max_solver_output;     /** Max. output of the solver. If one value exceeds this maximum, all other values will be scaled accordingly*/

    //Properties
    double min_eigenvalue;    /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double max_solver_output_norm;   /** Maximum norm of (J#) * y */

    //Helpers
    base::VectorXd tmp;
};
}
#endif

