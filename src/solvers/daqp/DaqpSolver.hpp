#ifndef WBC_SOLVERS_DAQP_SOLVER_HPP
#define WBC_SOLVERS_DAQP_SOLVER_HPP

#include "../../core/QPSolver.hpp"

#include <Eigen/Core>
#include <daqp/api.h>
#include <daqp/utils.h>

namespace wbc {

class HierarchicalQP;
struct QuadraticProgram;

/**
 * @brief The DaqpSolver class is a wrapper for the dual active-set QP solver DAQP (see https://github.com/darnstrom/daqp). DAQP solves problems of shape:
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{H}\mathbf{x}+\mathbf{x}^T\mathbf{f}& \\
 *             & & \\
 *        s.t. & \mathbf{lb} \leq \mathbf{x} \leq \mathbf{ub}& \\
 *             & \mathbf{lbA} \leq \mathbf{Ax} \leq \mathbf{ubA}& \\
 *        \end{array}
 *  \f]
 *
 * The constraints of the wbc::QuadraticProgram are mapped as follows: simple bounds (if any) first, then the
 * inequality constraints C, and finally the equality constraints A, which are added as active, immutable constraints
 * with equal lower and upper bounds. Between consecutive calls to solve() the working set of the previous solution
 * is used to warm start the solver, unless warm starting is disabled.
 *
 * Solver parameters
 *  - See DAQPSettings in daqp/types.h for all options and daqp_default_settings() for their default values.
 *    The settings can be changed with setSettings() and are applied at the next call to solve().
 *
 * Reference: D. Arnström, A. Bemporad, D. Axehill. A Dual Active-Set Solver for Embedded Quadratic Programming
 * Using Recursive LDL^T Updates. IEEE Transactions on Automatic Control 67(8), 2022.
 */
class DaqpSolver : public QPSolver{
private:
    static QPSolverRegistry<DaqpSolver> reg;

public:
    DaqpSolver();
    virtual ~DaqpSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve. Each vector entry correspond to a stage in the hierarchy where
     *                    the first entry has the highest priority. Currently only one priority level is implemented, an
     *                    std::invalid_argument is thrown otherwise.
     * @param solver_output solution of the quadratic program
     * @param allow_warm_start If true, the working set of the previous solution is used to warm start the solver
     */
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start = true);

    /** Get the current solver settings*/
    DAQPSettings getSettings() const { return settings; }

    /** Set the solver settings. They are applied at the next call to solve()*/
    void setSettings(const DAQPSettings& s);

    /** Set the maximum number of iterations*/
    void setMaxNIter(const int n);

    /** Get the maximum number of iterations*/
    int getMaxNIter() const { return settings.iter_limit; }

    /** Get the number of iterations performed in the last call to solve()*/
    int getNIter() const { return result.iter; }

    /** Get the exit flag of the last call to solve(). See DAQP_EXIT_* in daqp/constants.h*/
    int getExitFlag() const { return result.exitflag; }

    /** Get the dual variables of the last solution: simple bounds first (if any), then inequality constraints, then equality constraints*/
    const Eigen::VectorXd& getDualSolution() const { return lam; }

protected:
    typedef Eigen::Matrix<c_float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

    /** Free the internal DAQP workspace*/
    void freeWorkspace();
    /** Copy the given QP into the DAQP problem buffers*/
    void toDaqp(const wbc::QuadraticProgram& qp);

    DAQPProblem problem;
    DAQPWorkspace work;
    DAQPSettings settings;
    DAQPResult result;
    bool settings_changed;

    int n_var, n_bounds, n_in, n_eq;

    Eigen::MatrixXd H;          /** Hessian (symmetric, so storage order does not matter) */
    Eigen::VectorXd f;          /** Gradient */
    RowMajorMatrix A;           /** General constraints [C; A], row-major as expected by DAQP */
    Eigen::VectorXd bupper;     /** Upper bounds [upper_x; upper_y; b] */
    Eigen::VectorXd blower;     /** Lower bounds [lower_x; lower_y; b] */
    Eigen::VectorXi sense;      /** Constraint types (equalities: active and immutable) */
    Eigen::VectorXd x;          /** Primal solution */
    Eigen::VectorXd lam;        /** Dual solution */
};

}

#endif
