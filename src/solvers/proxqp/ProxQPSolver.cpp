#include "ProxQPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <Eigen/Core>
#include <iostream>

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/status.hpp>

namespace wbc {

QPSolverRegistry<ProxQPSolver> ProxQPSolver::reg("proxqp");

ProxQPSolver::ProxQPSolver()
{
}

/// solve problem:
/// min  0.5 * x'Hx + g'x
/// s.t. Ax = b
///      l < Cx < u 
void ProxQPSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start)
{
    namespace pqp = proxsuite::proxqp;

    assert(hierarchical_qp.size() == 1);

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];
    assert(qp.isValid());

    
    size_t n_var = qp.A.cols();
    size_t n_eq = qp.neq;
    size_t n_in = qp.nin + qp.lower_x.size();

    // merge ineuqalities and bounds together
    _C_mtx.resize(n_in, n_var);
    _l_vec.resize(n_in);
    _u_vec.resize(n_in);

    _C_mtx.topRows(qp.nin) = qp.C;
    _C_mtx.bottomRows(qp.lower_x.size()).setIdentity();
    _l_vec << qp.lower_y, qp.lower_x;
    _u_vec << qp.upper_y, qp.upper_x;

    if(!allow_warm_start)
        configured = false;

    if(!configured) 
    {
        _solver_ptr = std::make_shared<pqp::dense::QP<double>>(n_var, n_eq, n_in);
        _solver_ptr->init(qp.H, qp.g, qp.A, qp.b, _C_mtx, _l_vec, _u_vec);
        _solver_ptr->settings = settings;

        configured = true;
    }
    else 
    {
        _solver_ptr->settings.initial_guess = pqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        _solver_ptr->update(qp.H, qp.g, qp.A, qp.b, _C_mtx, _l_vec, _u_vec);
    }

    _solver_ptr->solve();
    
    solver_output.resize(qp.nq);
    solver_output = _solver_ptr->results.x;

    auto status = _solver_ptr->results.info.status;

    if(status == pqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED)
        throw std::runtime_error("ProxQP returned error status: max iterations reached.");
    if(status == pqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is primal infeasible.");
    if(status == pqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is dual infeasible.");

    _actual_n_iter = _solver_ptr->results.info.iter;

}

} // namespace wbc
