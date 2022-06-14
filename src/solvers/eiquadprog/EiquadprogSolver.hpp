#ifndef WBC_SOLVERS_EIQUADPROG_SOLVER_HPP
#define WBC_SOLVERS_EIQUADPROG_SOLVER_HPP

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
    void setMaxNoWSR(const uint& n){ _n_wsr = n; }
    
    /** Get the maximum number of working set recalculations to be performed during the initial homotopy*/
    uint getMaxNoWSR(){ return _n_wsr; }
    
    /** Retrieve the return value from the last QP calculation*/
    //qpOASES::returnValue getReturnValue();
    /** Get number of working set recalculations actually performed*/
    int getNoWSR(){ return _actual_n_wsr; }
    
    /** Return current solver options*/
    //qpOASES::Options getOptions(){return options;}
    /** Set new solver options*/
    //void setOptions(const qpOASES::Options& opt);
    
    /** Get Quadratic program*/
    //const qpOASES::SQProblem& getSQProblem(){return sq_problem;}

protected:
    //qpOASES::Options options;

    eiquadprog::solvers::EiquadprogFast _solver;
    
    int _n_wsr;
    int _actual_n_wsr;

    //qpOASES::returnValue ret_val;

    Eigen::MatrixXd _CE_mtx;
    Eigen::VectorXd _ce0_vec;

    Eigen::MatrixXd _CI_mtx;
    Eigen::VectorXd _ci0_vec;

    base::Time _stamp;
};

}

#endif
