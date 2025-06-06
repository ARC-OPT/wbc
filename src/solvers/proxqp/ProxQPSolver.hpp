#ifndef WBC_SOLVERS_PROXQP_SOLVER_HPP
#define WBC_SOLVERS_PROXQP_SOLVER_HPP

#include "../../core/QPSolver.hpp"

#include <memory>
#include <iostream>

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
 *
 * Reference:
 * Antoine Bambade, Sarah El-Kazdadi, Adrien Taylor, Justin Carpentier. PROX-QP: Yet another Quadratic Programming Solver for
 * Robotics and beyond. RSS 2022 - Robotics: Science and Systems, Jun 2022, New York, United States. ⟨hal-03683733⟩
 *
 * Parameters:
 *  - See proxsuite::proxqp::Setting in settings.h for all possible options and default values.
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
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start = true);

    /** Get number of working set recalculations actually performed*/
    int getNter(){ return _actual_n_iter; }

    /** Set Solver options. Has to be called before first call to solve()!*/
    void setOptions(proxsuite::proxqp::Settings<double> opt){settings = opt;}

protected:

    std::shared_ptr<proxsuite::proxqp::dense::QP<double>> _solver_ptr;
    int _actual_n_iter;

    Eigen::MatrixXd _C_mtx; // inequalities matrix (including bounds)
    Eigen::VectorXd _l_vec; // inequalities lower bounds
    Eigen::VectorXd _u_vec; // inequalities upper bounds

    proxsuite::proxqp::Settings<double> settings;
};

}

#endif
