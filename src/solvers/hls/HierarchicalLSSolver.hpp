#ifndef WBC_SOLVERS_HIERARCHICAL_LS_SOLVER_HPP
#define WBC_SOLVERS_HIERARCHICAL_LS_SOLVER_HPP

#include <base/Eigen.hpp>
#include <vector>
#include "../../core/QPSolver.hpp"

namespace wbc{

class HierarchicalQP;

/**
 * @brief Implementation of the hierarchical weighted damped least squares solver (HWLS), similar to
 * Schutter, J. et al. “Constraint-based Task Specification and Estimation for Sensor-Based Robot Systems in the Presence of Geometric Uncertainty.” The International Journal of Robotics Research 26 (2007): 433 - 455.
 *  It solves the following optimization problem
 *  \f[
 *        \begin{array}{ccc}
 *        min(\dot{\mathbf{q}}) & ||\dot{\mathbf{q}}||_2 \\
 *             & & \\
 *        s.t. &  \mathbf{A}_{w,1}  \dot{\mathbf{q}} = \dot{\mathbf{x}}_1 & \\
 *             &  \mathbf{A}_{w,2}  \dot{\mathbf{q}} = \dot{\mathbf{x}}_2 & \\
 *             &   ... & \\
 *             &  \mathbf{A}_{w,N}  \mathbf{\dot{q}} = \dot{\mathbf{x}}_N & \\
 *        \end{array}
 *  \f]
 *
 * where A is the weighted constraint matrix of priority level i, \f$\dot{\mathbf{q}}\f$ the robot joint velocities, \f$\dot{\mathbf{x}}_i\f$ the desired task space velocities
 * of priority level i.
 * The solver ensures a hierarchy between the different tasks using nullspace projections. That is, the equation system with the highest priority will be solved fully if (n_rows <= n_cols),
 * the eqn. system of the next priority will be solved in the nullspace of the priovious priority, and so on. Additionally the solver can include weights in joint space and task space.
 */
class HierarchicalLSSolver : public QPSolver{
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

    HierarchicalLSSolver();
    virtual ~HierarchicalLSSolver();

    /**
     * @brief configure Resizes member variables
     * @param constraint_variables_per_prio Number of constraint variables per priority, i.e. number of row of the constraint Jacobian of that priority
     * @param no_of_joints Number of robot joints
     * @return true in case of successful initialization, false otherwise
     */
    bool configure(const std::vector<int>& n_constraints_per_prio, const unsigned int n_joints);

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output);

    /**
     * @brief setJointWeights Sets the joint weight vector for all priorities
     * @param weights Has to have same size as number of joints. Values have to be >= 0! A values of 0 means that the joint does not
     *                contribute to the solution at all.
     */
    void setJointWeights(const base::VectorXd& weights);

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

    /** Return the min eigenvalue term.*/
    double getMinEigenvalue(){return min_eigenvalue;}

    /**
     * @brief setMaxSolverOutputNorm Sets the maximum norm term. The solution of the solver will have a norm that is below this value.
     *        This value will be used to compute a suitable damping factor for matrix inversion.
     * @param norm_max Maximum output norm. Has to be > 0!
     */
    void setMaxSolverOutputNorm(double norm_max);

    /** Return the maximum norm term.*/
    double getMaxSolverOutputNorm(){return max_solver_output_norm;}

    /**
     * @brief Has configure() been  called already?
     */
    bool isConfigured(){return configured;}

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

    //Properties
    double min_eigenvalue;    /** Precision for eigenvalue inversion. Inverse of an Eigenvalue smaller than this will be set to zero*/
    double max_solver_output_norm;   /** Maximum norm of (J#) * y */

    //Helpers
    base::VectorXd tmp;
};
}
#endif

