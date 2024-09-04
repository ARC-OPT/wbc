#ifndef WBC_OSQP_SOLVER_HPP
#define WBC_OSQP_SOLVER_HPP

#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>
#include "../../core/QPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"

namespace wbc {

/**
 * @brief The OsqpSolver solves convex quadratic programs (QPs) of the form
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{P}\mathbf{x}+\mathbf{q}^T\mathbf{x}& \\
 *             & & \\
 *        s.t. & \mathbf{l} \leq Ax \leq u& \\
 *        \end{array}
 *  \f]
 *
 * The solver runs the following ADMM algorithm, which is described in
 * Stellato, B., Banjac, G., Goulart, P. et al. OSQP: an operator splitting solver for quadratic programs.
 * Math. Prog. Comp. 12, 637–672 (2020). https://doi.org/10.1007/s12532-020-00179-2
 *
 * Solver Parameters:
 *  - Check OSQPSettings Struct in file osqp_api_types.h to see all parameters
 *  - Check osqp_api_constants.h for the default settings
 *  - To change parameters you can access the public solver variable:
 *       solver.settings()->setXY()
 */
class OsqpSolver : public QPSolver{
private:
    static QPSolverRegistry<OsqpSolver> reg;
public:
    OsqpSolver();
    ~OsqpSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the (hierarchical) quadratic program to solve.
     * @param solver_output solution of the quadratic program as vector
     */
    virtual void solve(const HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start = true);

    /** The osqp wrapper variable*/
    OsqpEigen::Solver solver;

protected:
    bool configured;

    Eigen::MatrixXd hessian_dense;
    Eigen::SparseMatrix<double> hessian_sparse;
    Eigen::MatrixXd constraint_mat_dense;
    Eigen::SparseMatrix<double> constraint_mat_sparse;
    Eigen::VectorXd gradient;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    void resize(uint nq, uint nc);
    std::string exitFlagToString(OsqpEigen::ErrorExitFlag flag){
        switch(flag){
            case OsqpEigen::ErrorExitFlag::DataValidationError: return "DataValidationError";
            case OsqpEigen::ErrorExitFlag::SettingsValidationError: return "SettingsValidationError";
            case OsqpEigen::ErrorExitFlag::LinsysSolverLoadError: return "LinsysSolverLoadError";
            case OsqpEigen::ErrorExitFlag::LinsysSolverInitError: return "LinsysSolverInitError";
            case OsqpEigen::ErrorExitFlag::NonCvxError: return "NonCvxError";
            case OsqpEigen::ErrorExitFlag::MemAllocError: return "MemAllocError";
            case OsqpEigen::ErrorExitFlag::WorkspaceNotInitError: return "WorkspaceNotInitError";
        default: return "NoError";
        }
    }
};

}

#endif
