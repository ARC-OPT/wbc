#ifndef WBC_SOLVERS_EIQUADPROG_SOLVER_HPP
#define WBC_SOLVERS_EIQUADPROG_SOLVER_HPP

#include "../../core/QPSolver.hpp"

#include <eiquadprog/eiquadprog-fast.hpp>

namespace wbc {

class HierarchicalQP;

/**
 * @brief The EiquadprogSolver class is a wrapper for the qp-solver eiquadprog (see https://github.com/stack-of-tasks/eiquadprog). It solves problems of shape:
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{G}\mathbf{x}+\mathbf{x}^T\mathbf{g0}& \\
 *             & & \\
 *        s.t. & \mathbf{CE}x + ce0 = 0& \\
 *             & \mathbf{CI}x + ci0 \geq 0& \\
 *        \end{array}
 *  \f]
 *
 * The implements the algorithm of Goldfarb and Idnani for the solution of a (convex) Quadratic Programming problem:
 * D. Goldfarb, A.  Idnani. A numerically stable dual method for solving strictly convex quadratic programs. Mathematical
 * Programming 27 (1983) pp. 1-33.
 *
 * Solver parameters
 *  - MaxNIter: Maximum number of working set recalculations to be performed during the initial homotopy
 */
class EiquadprogSolver : public QPSolver{
private:
    static QPSolverRegistry<EiquadprogSolver> reg;

public:
    EiquadprogSolver();
    virtual ~EiquadprogSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param constraints Description of the hierarchical quadratic program to solve. Each vector entry correspond to a stage in the hierarchy where
     *                    the first entry has the highest priority. Currently only one priority level is implemented.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start = true);

    /** Set the maximum number of working set recalculations to be performed during the initial homotopy*/
    void setMaxNIter(const uint& n){ _n_iter = n; }
    
    /** Get the maximum number of working set recalculations to be performed during the initial homotopy*/
    uint getMaxNIter(){ return _n_iter; }

    /** Get number of working set recalculations actually performed*/
    int getNter(){ return _actual_n_iter; }

protected:
    eiquadprog::solvers::EiquadprogFast _solver;
    
    int _n_iter;
    int _actual_n_iter;

    Eigen::MatrixXd _CI_mtx;
    Eigen::VectorXd _ci0_vec;
};

}

#endif
