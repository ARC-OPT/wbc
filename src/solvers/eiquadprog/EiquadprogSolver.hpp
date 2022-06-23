#ifndef WBC_SOLVERS_EIQUADPROG_SOLVER_HPP
#define WBC_SOLVERS_EIQUADPROG_SOLVER_HPP

#include "../../core/QPSolverFactory.hpp"
#include "../../core/QPSolver.hpp"

#include <base/Time.hpp>

#include <eiquadprog/eiquadprog-fast.hpp>

namespace wbc {

class HierarchicalQP;

/**
 * @brief The QPOASESSolver class is a wrapper for the qp-solver qpoases (see https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf). It solves problems of shape:
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{H}\mathbf{x}+\mathbf{x}^T\mathbf{g}& \\
 *             & & \\
 *        s.t. & lb(\mathbf{Ax}) \leq \mathbf{Ax} \leq ub(\mathbf{Ax})& \\
 *             & lb(\mathbf{x}) \leq \mathbf{x} \leq ub(\mathbf{x})& \\
 *        \end{array}
 *  \f]
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
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output);

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

    Eigen::MatrixXd _CE_mtx;
    Eigen::VectorXd _ce0_vec;

    Eigen::MatrixXd _CI_mtx;
    Eigen::VectorXd _ci0_vec;
};

}

#endif
