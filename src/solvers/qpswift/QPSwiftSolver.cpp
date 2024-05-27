#include "QPSwiftSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base-logging/Logging.hpp>

#include <iostream>

namespace wbc {

QPSolverRegistry<QPSwiftSolver> QPSwiftSolver::reg("qpswift");

QPSwiftSolver::QPSwiftSolver(){
    my_qp = 0;
    max_iter = 1000;
    rel_tol = 1e-5;
    abs_tol = 1e-5;
    sigma = 100;
    verbose_level = 0;
}

QPSwiftSolver::~QPSwiftSolver(){
    if(my_qp)
        QP_CLEANUP_dense(my_qp);
}

void QPSwiftSolver::toQpSwift(const wbc::QuadraticProgram &qp){

    G.setZero();

    P = qp.H;
    c = qp.g;
    A = qp.A;
    b = qp.b;

    // create inequalities matrix (inequalities constraints + bounds)
    G.middleRows(0, qp.nin) = qp.C;
    G.middleRows(qp.nin, qp.nin) = -qp.C;
    h.segment(0, qp.nin) = qp.upper_y;
    h.segment(qp.nin, qp.nin) = -qp.lower_y;

    if(qp.bounded){
        G.middleRows(2*qp.nin, n_dec).diagonal().setConstant(1.0);        // map bounds as inequalities
        G.middleRows(2*qp.nin+n_dec, n_dec).diagonal().setConstant(-1.0);
        h.segment(2*qp.nin, n_dec) = qp.upper_x;        // map bounds as inequalities
        h.segment(2*qp.nin+n_dec, n_dec) = -qp.lower_x;
    }

    my_qp = QP_SETUP_dense(n_dec,                   // Number decision variables
                           n_ineq,                  // Number inequality constraints
                           n_eq,                    // Number equality constraints
                           (double*)P.data(),       // Hessian matrix
                           (double*)A.data(),       // Equality constraint matrix
                           (double*)G.data(),       // Inequality constraint matrix
                           (double*)c.data(),       // Cost function gradient vector
                           (double*)h.data(),       // Inequality constraint vector
                           (double*)b.data(),       // Equality constraint vector
                           NULL,
                           COLUMN_MAJOR_ORDERING);

    my_qp->options->maxit = max_iter;
    my_qp->options->reltol = rel_tol;
    my_qp->options->abstol = abs_tol;
    my_qp->options->sigma = sigma;
    my_qp->options->verbose = verbose_level;
    LOG_DEBUG_S << "Setup Time     : " << my_qp->stats->tsetup * 1000.0 << " ms" << std::endl;
}

void QPSwiftSolver::solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output){

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("QPSwiftSolver::solve: Number of task hierarchies must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];

    if(n_dec != qp.nq || n_eq != qp.neq || n_ineq != qp.nin)
        configured = false;

    if(!configured){
        // Count equality / inequality constraints
        n_dec = qp.nq;
        n_eq = qp.neq;
        n_ineq = 2 * (qp.nin + qp.upper_x.size());

        A.resize(n_eq, n_dec);
        b.resize(n_eq);
        G.resize(n_ineq, n_dec);
        h.resize(n_ineq);
        P.resize(n_dec, n_dec);
        c.resize(n_dec);
        solver_output.resize(n_dec);

        LOG_DEBUG_S << "n_dec:    " << n_dec    << std::endl;
        LOG_DEBUG_S << "n_eq:     " << n_eq     << std::endl;
        LOG_DEBUG_S << "n_ineq:   " << n_ineq   << std::endl;
        configured = true;
    }

    toQpSwift(qp);

    qp_int exit_code = QP_SOLVE(my_qp);

    switch(exit_code){
    case QP_OPTIMAL:{
        LOG_DEBUG_S << "Solve Time     : " << (my_qp->stats->tsolve + my_qp->stats->tsetup) * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "KKT_Solve Time : " << my_qp->stats->kkt_time * 1000.0  << " ms" << std::endl;
        LOG_DEBUG_S << "LDL Time       : " << my_qp->stats->ldl_numeric * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "Diff	       : " << (my_qp->stats->kkt_time - my_qp->stats->ldl_numeric) * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "Iterations     : " << my_qp->stats->IterationCount << std::endl;
        LOG_DEBUG_S << "QPSwiftSolver: Optimal Solution Found" << std::endl;
        break;
    }
    case QP_MAXIT:{
        LOG_DEBUG_S << "Solve Time     : " << my_qp->stats->tsolve * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "KKT_Solve Time : " << my_qp->stats->kkt_time * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "LDL Time       : " << my_qp->stats->ldl_numeric * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "Diff	       : " << (my_qp->stats->kkt_time - my_qp->stats->ldl_numeric) * 1000.0 << " ms" << std::endl;
        LOG_DEBUG_S << "Iterations     : " << my_qp->stats->IterationCount << std::endl;
        throw std::runtime_error("QPSwiftSolver failed: Maximum Iterations reached");
    }
    case QP_FATAL:{
        throw std::runtime_error("QPSwiftSolver failed: Unknown error");
    }
    case QP_KKTFAIL:{
        throw std::runtime_error("QPSwiftSolver failed: LDL Factorization failed");
    }
    }

    for(int i = 0; i < n_dec; i++)
        solver_output[i] = my_qp->x[i];
}

}
