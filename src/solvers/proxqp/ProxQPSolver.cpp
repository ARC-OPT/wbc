#include "ProxQPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/status.hpp>

namespace wbc {

QPSolverRegistry<ProxQPSolver> ProxQPSolver::reg("proxqp");

ProxQPSolver::ProxQPSolver()
{
    _n_iter = 10000;
    _eps_abs = 1e-9;
}

/// solve problem:
/// min  0.5 * x'Hx + g'x
/// s.t. Ax = b
///      l < Cx < u 
void ProxQPSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output)
{
    namespace pqp = proxsuite::proxqp;

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("ProxQPSolver::solve: Constraints vector size must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];
    qp.check();

    
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

    if(!configured) 
    {
        _n_var_init = n_var;
        _n_eq_init = n_eq;
        _n_in_init = n_in;

        _solver_ptr = std::make_shared<pqp::dense::QP<double>>(n_var, n_eq, n_in);
        _solver_ptr->settings.eps_abs = _eps_abs;
        _solver_ptr->settings.max_iter = _n_iter;
        // _solver_ptr->settings.eps_primal_inf = 1e-6;
        // _solver_ptr->settings.preconditioner_max_iter = 100;
        // _solver_ptr->settings.initial_guess = pqp::InitialGuessStatus::NO_INITIAL_GUESS;

        _solver_ptr->init(qp.H, qp.g, qp.A, qp.b, _C_mtx, _l_vec, _u_vec);

        configured = true;
    }
    else 
    {
        if(n_var != _n_var_init)
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
        if(n_in != _n_in_init)
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");

        _solver_ptr->settings.initial_guess = pqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        _solver_ptr->update(qp.H, qp.g, qp.A, qp.b, _C_mtx, _l_vec, _u_vec);
    }

//     std::cerr << "qp.nq = " << qp.nq <<std::endl;
//     std::cerr << "qp.neq = " << qp.neq <<std::endl;
//     std::cerr << "qp.nin = " << qp.nin <<std::endl;
//     std::cerr << "qp.bounded = " << qp.bounded <<std::endl;
//     std::cerr << "l: " <<_l_vec.transpose() <<std::endl;
//     std::cerr << "u: "<<_u_vec.transpose() <<std::endl;
//     std::cerr << "C:\n"<<_C_mtx <<std::endl;
//     std::cerr << "A:\n"<< qp.A << std::endl;
//     std::cerr << "b: "<< qp.b << std::endl;
//     std::cerr << "eps_abs: " << _solver_ptr->settings.eps_abs << std::endl;
//     std::cerr << "max_iter: " << _solver_ptr->settings.max_iter << std::endl;

    _solver_ptr->solve();
    
    solver_output.resize(qp.nq);
    solver_output = _solver_ptr->results.x;

    auto status = _solver_ptr->results.info.status;

    // if(status == pqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED)
    //     std::cerr << "ProxQP returned error status: max iterations reached." << std::endl;
    // if(status == pqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE)
    //     std::cerr << "ProxQP returned error status: problem is primal infeasible." << std::endl;
    // if(status == pqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE)
    //     std::cerr << "ProxQP returned error status: problem is dual infeasible." << std::endl;

    if(status == pqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED)
        throw std::runtime_error("ProxQP returned error status: max iterations reached.");
    if(status == pqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is primal infeasible.");
    if(status == pqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE)
        throw std::runtime_error("ProxQP returned error status: problem is dual infeasible.");

    _actual_n_iter = _solver_ptr->results.info.iter;
}

} // namespace wbc
