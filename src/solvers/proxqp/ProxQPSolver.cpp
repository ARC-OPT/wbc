#include "ProxQPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace wbc {

QPSolverRegistry<ProxQPSolver> ProxQPSolver::reg("ProxQP");

ProxQPSolver::ProxQPSolver()
{
    _n_iter = 100;
    _eps_abs = 1e-9;
}

/// solve problem:
/// min  0.5 * x'Hx + g'x
/// s.t. Ax = b
///      l < Cx < u 
void ProxQPSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output)
{
    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("ProxQPSolver::solve: Constraints vector size must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];
    qp.check();

    
    size_t n_var = qp.A.cols();
    size_t n_eq = qp.neq;
    size_t n_in = qp.nin + qp.lower_x.size();

    if(!configured) 
    {
        _solver = pqp::Dense<double>(n_var, n_eq, n_in);
        _solver.settings.eps_abs = _eps_abs;
        _solver.settings.max_iter = _n_iter;

        // hessian, gradient and equalities matrices in qp are ok
        // configuring inequalities constraints matrix
        _C_mtx.resize(n_in, n_var);
        _l_vec.resize(n_in);
        _u_vec.resize(n_in);

        configured = true;
    }
    else 
    {
        if(n_var != _C_mtx.cols())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
        if(n_con != _C_mtx.rows())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
    }

    // merge ineuqalities and bounds together

    _C_mtx.topRows(qp.nin) = qp.C;
    _C_mtx.bottomRows(qp.lower_x.size()).setIdentity();
    
    _l_vec << qp.lower_y, qp.lower_x;
    _u_vec << qp.upper_y, qp.upper_x;

    _solver.init(qp.H, qp.g, qp.A, qp.b, _C_mtx, _u_vec, _l_vec);
    _solver.solve();
    
    solver_output.resize(qp.nq);
    solver_output = _solver.results.x;

    auto status = _solver.results.info.status;

    if(status == pqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED)
        throw std::runtime_error("ProxQP returned error status: max iterations reached.");
    if(status == pqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is primal infeasible.");
    if(status == pqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is dual infeasible.");

    _actual_n_iter = _solver.results.info.iter;
}
}
