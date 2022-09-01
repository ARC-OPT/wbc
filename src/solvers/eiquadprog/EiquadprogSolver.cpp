#include "EiquadprogSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>

namespace wbc {

QPSolverRegistry<EiquadprogSolver> EiquadprogSolver::reg("eiquadprog");

EiquadprogSolver::EiquadprogSolver()
{
    _n_iter = 100;
}

EiquadprogSolver::~EiquadprogSolver()
{

}

void EiquadprogSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output)
{

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("EiquadprogSolver::solve: Constraints vector size must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];
    qp.check();

    size_t n_in = 2 * (qp.nin + qp.nq);
    size_t n_eq = qp.neq;
    size_t n_var = qp.nq;

    if(!configured) 
    {
        _solver.reset(n_var, n_eq, n_in);
        _solver.setMaxIter(_n_iter);

        // hessian and gradient are ok (don#t need to be stacked)
        // equality contraint is ok also
        // configuring  inequalities constraints matrices
        _CI_mtx.resize(n_in, n_var);
        _ci0_vec.resize(n_in);

        configured = true;
    }
    else 
    {
        if(n_var != _CI_mtx.cols())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
        if(n_in != _CI_mtx.rows())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
    }

    _CI_mtx.setZero();

    // create inequalities constraint matric (inequalities + bounds)
    _CI_mtx.middleRows(0, qp.nin) = qp.C;
    _CI_mtx.middleRows(qp.nin, qp.nin) = -qp.C;
    _CI_mtx.middleRows(2*qp.nin, n_var).diagonal().setConstant(1.0);        // map bounds as inequalities
    _CI_mtx.middleRows(2*qp.nin+n_var, n_var).diagonal().setConstant(-1.0);
    
    _ci0_vec << -qp.lower_y, qp.upper_y, -qp.lower_x, qp.upper_x;

    namespace eq = eiquadprog::solvers;

    Eigen::VectorXd out(qp.nq);
    eq::EiquadprogFast_status status = _solver.solve_quadprog(
        qp.H, qp.g, qp.A, -qp.b, _CI_mtx, _ci0_vec, out);
    
    solver_output.resize(qp.nq);
    solver_output = out;

    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_UNBOUNDED){
        qp.print();
        throw std::runtime_error("Eiquadprog returned error status:unbounded.");
    }
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_MAX_ITER_REACHED){
        qp.print();
        throw std::runtime_error("Eiquadprog returned error status: max iterations reached.");
    }
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_REDUNDANT_EQUALITIES){
        qp.print();
        throw std::runtime_error("Eiquadprog returned error status: redundant equalities.");
    }
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_INFEASIBLE){
        qp.print();
        throw std::runtime_error("Eiquadprog returned error status: infeasible.");
    }

    _actual_n_iter = _solver.getIteratios();
}
}
