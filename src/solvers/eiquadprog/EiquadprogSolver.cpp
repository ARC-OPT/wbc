#include "EiquadprogSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>


namespace wbc {

EiquadprogSolver::EiquadprogSolver()
{
    _n_wsr = 100;
    //options.setToDefault();
}

EiquadprogSolver::~EiquadprogSolver()
{

}

void EiquadprogSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output)
{

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("EiquadprogSolver::solve: Constraints vector size must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];

    // Joint space upper and lower bounds
    if(qp.lower_x.size() > 0 && qp.lower_x.size() != qp.nq)
        throw std::runtime_error("Number of joints in quadratic program is " + std::to_string(qp.nq)
            + ", but lower bound has size " + std::to_string(qp.lower_x.size()));

    if(qp.upper_x.size() > 0 && qp.upper_x.size() != qp.nq)
        throw std::runtime_error("Number of joints in quadratic program is " + std::to_string(qp.nq)
            + ", but lower bound has size " + std::to_string(qp.upper_x.size()));

    // Constraint space upper and lower bounds
    if(qp.lower_y.size() > 0 && qp.lower_y.size() != qp.nc)
        throw std::runtime_error("Number of constraints in quadratic program is " + std::to_string(qp.nc)
            + ", but lower bound has size " + std::to_string(qp.lower_y.size()));

    if(qp.upper_y.size() > 0 && qp.upper_y.size() != qp.nc)
        throw std::runtime_error("Number of constraints in quadratic program is " + std::to_string(qp.nc)
            + ", but lower bound has size " + std::to_string(qp.upper_y.size()));

    // Constraint matrix
    if(qp.A.rows() != qp.nc || qp.A.cols() != qp.nq)
        throw std::runtime_error("Constraint matrix A should have size " + std::to_string(qp.nc) + "x" + std::to_string(qp.nq) +
                                 "but has size " +  std::to_string(qp.A.rows()) + "x" + std::to_string(qp.A.cols()));

    // Hessian matrix:
    if(qp.H.rows() != qp.nq || qp.H.cols() != qp.nq)
        throw std::runtime_error("Hessian matrix H should have size " + std::to_string(qp.nq) + "x" + std::to_string(qp.nq) +
                                 "but has size " +  std::to_string(qp.H.rows()) + "x" + std::to_string(qp.H.cols()));

    // Gradient vector
    if(qp.g.size() > 0 && qp.g.size() != qp.nq)
        throw std::runtime_error("Gradient vector g should have size " + std::to_string(qp.nq) + "but has size " + std::to_string(qp.g.size()));


    size_t n_con = qp.lower_x.size() + qp.upper_x.size() + qp.lower_y.size() + qp.upper_y.size();
    size_t n_var = qp.A.cols();

    if(!configured) 
    {
        _solver.reset(n_var, 0, n_con);
        _solver.setMaxIter(_n_wsr);

        // hessian and gradient are ok (don#t need to be stacked)
        // configuring constraints matrices
        _CE_mtx.resize(0, n_var);
        _ce0_vec.resize(0);
        _CI_mtx.resize(n_con, n_var);
        _ci0_vec.resize(n_con);

        configured = true;
    }
    else 
    {
        if(n_var != _CI_mtx.cols())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
        if(n_con != _CI_mtx.rows())
            throw std::runtime_error("QP problem changed dynamically. Not supported at the moment.");
    }

    size_t ci_cnt = 0; // count of constraints already set

    _CI_mtx.block(ci_cnt, 0, qp.lower_x.size(), n_var) = Eigen::MatrixXd::Identity(qp.lower_x.size(), n_var);
    _ci0_vec.segment(ci_cnt, qp.lower_x.size()) = -qp.lower_x;
    ci_cnt += qp.lower_x.size();

    _CI_mtx.block(ci_cnt, 0, qp.upper_x.size(), n_var) = -Eigen::MatrixXd::Identity(qp.upper_x.size(), n_var);
    _ci0_vec.segment(ci_cnt, qp.upper_x.size()) = qp.upper_x;
    ci_cnt += qp.upper_x.size();

    _CI_mtx.block(ci_cnt, 0, qp.lower_y.size(), n_var) = qp.A;
    _ci0_vec.segment(ci_cnt, qp.lower_y.size()) = -qp.lower_y;
    ci_cnt += qp.lower_y.size();

    _CI_mtx.block(ci_cnt, 0, qp.upper_y.size(), n_var) = -qp.A;
    _ci0_vec.segment(ci_cnt, qp.upper_y.size()) = qp.upper_y;
    ci_cnt += qp.upper_y.size();

    namespace eq = eiquadprog::solvers;

    Eigen::VectorXd out(qp.nq);
    
    eq::EiquadprogFast_status status = _solver.solve_quadprog(
        qp.H, qp.g, _CE_mtx, _ce0_vec, _CI_mtx, _ci0_vec, out);
    
    solver_output.resize(qp.nq);
    solver_output = out;

    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_UNBOUNDED)
        throw std::runtime_error("Eiquadprog returned error status:unbounded.");
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_MAX_ITER_REACHED)
        throw std::runtime_error("Eiquadprog returned error status: max iterations reached.");
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_REDUNDANT_EQUALITIES)
        throw std::runtime_error("Eiquadprog returned error status: redundant equalities.");
    if(status == eq::EiquadprogFast_status::EIQUADPROG_FAST_INFEASIBLE)
        throw std::runtime_error("Eiquadprog returned error status: infeasible.");

    _actual_n_wsr = _solver.getIteratios();
}

/*
returnValue EiquadprogSolver::getReturnValue()
{
    return ret_val;
}


void EiquadprogSolver::setOptions(const qpOASES::Options& opt)
{
    options = opt;
    sq_problem.setOptions(opt);
}
*/

}
