#ifndef WBC_SOLVERS_PROXQP_SOLVER_HPP
#define WBC_SOLVERS_PROXQP_SOLVER_HPP

#include "../../core/QPSolver.hpp"

#include <memory>

#include <base/Time.hpp>

#include <proxsuite/proxqp/dense/wrapper.hpp>

namespace wbc {

class HierarchicalQP;

/**
 * @brief The ProxQPSolver class is a wrapper for the qp-solver prox-qp (see https://github.com/Simple-Robotics/proxsuite). It solves problems of shape:
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{H}\mathbf{x}+\mathbf{x}^T\mathbf{g}& \\
 *             & & \\
 *        s.t. & \mathbf{Ax} = \mathbf{b}& \\
 *             & \mathbf{l} \leq \mathbf{Cx} \leq \mathbf{u}& \\
 *        \end{array}
 *  \f]
 */
class ProxQPSolver : public QPSolver{
private:
    static QPSolverRegistry<ProxQPSolver> reg;

public:
    ProxQPSolver();
    virtual ~ProxQPSolver() noexcept { };

    /**
     * @brief solve Solve the given quadratic program
     * @param constraints Description of the hierarchical quadratic program to solve. Each vector entry correspond to a stage in the hierarchy where
     *                    the first entry has the highest priority. Currently only one priority level is implemented.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output);

    /** Set the maximum number of working set recalculations to be performed during the initial homotopy*/
    void setMaxNIter(const uint& n){ _n_iter = n; }

    /** Get the maximum number of working set recalculations to be performed during the initial homotopy*/
    uint getMaxNIter(){ return _n_iter; }

    /** Get number of working set recalculations actually performed*/
    int getNter(){ return _actual_n_iter; }

protected:

    std::shared_ptr<proxsuite::proxqp::dense::QP<double>> _solver_ptr;

    double _eps_abs = 1e-9;
    int _n_iter;
    int _actual_n_iter;

    size_t _n_var_init; // number of variables in the configured solver instance
    size_t _n_eq_init;  // number of equalities in the configured solver instance
    size_t _n_in_init;  // number of inequalities in the configured solver instance (inclusing bounds)

    Eigen::MatrixXd _C_mtx; // inequalities matrix (including bounds)
    Eigen::VectorXd _l_vec; // inequalities lower bounds
    Eigen::VectorXd _u_vec; // inequalities upper bounds
};

}

#endif
