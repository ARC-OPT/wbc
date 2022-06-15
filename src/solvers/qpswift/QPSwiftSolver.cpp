#include "QPSwiftSolver.hpp"
#include <core/QuadraticProgram.hpp>
#include <base-logging/Logging.hpp>

namespace wbc {

QPSwiftSolver::QPSwiftSolver(){
    my_qp = 0;
}

QPSwiftSolver::~QPSwiftSolver(){
    if(my_qp)
        QP_CLEANUP_dense(my_qp);
}

void QPSwiftSolver::toQpSwift(const wbc::QuadraticProgram &qp){
    A.setZero();
    G.setZero();
    h.setZero();
    b.setZero();

    int j = 0, k = 0;
    for(uint i = 0; i < qp.lower_y.size(); i++){
        if(qp.lower_y[i] == qp.upper_y[i]){ // Equality constraints
            A.row(j) = qp.A.row(i);
            b[j++]   = qp.lower_y[i];
        }
        else{  // Inequality constraints
            G.row(k)   = qp.A.row(i);
            h[k++]     = qp.upper_y[i];
            G.row(k)   = -qp.A.row(i);
            h[k++]     = -qp.lower_y[i];
        }
    }
    // Map bounds to Inequality constraints
    for(uint i = 0; i < n_bounds; i++){
        G(k,i) = 1.0;
        h[k++] = qp.upper_x[i];
        G(k,i) = -1.0;
        h[k++] = -qp.lower_x[i];
    }
    P = qp.H;
    c = qp.g;

}

void QPSwiftSolver::solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output){

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("QPSwiftSolver::solve: Number of task hierarchies must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];

    if(!configured){        
        // Count equality / inequality constraints
        n_dec = qp.nq, n_ineq = 0, n_eq = 0, n_bounds = qp.lower_x.size();
        for(uint i = 0; i < qp.lower_y.size(); i++){
            if(qp.lower_y[i] == qp.upper_y[i])
                n_eq++;
            else
                n_ineq+=2; // Inequality constraints require 2 entries, one for the upper and one for the lower bound
        }
        n_ineq += 2*n_bounds; // Model bounds on the decision variables as inequality constraints here

        A.resize(n_eq,n_dec);
        b.resize(n_eq);
        G.resize(n_ineq,n_dec);
        h.resize(n_ineq);
        P.resize(n_dec,n_dec);
        c.resize(n_dec);
        solver_output.resize(n_dec);

        LOG_DEBUG_S << "n_dec:    " << n_dec    << std::endl;
        LOG_DEBUG_S << "n_eq:     " << n_eq     << std::endl;
        LOG_DEBUG_S << "n_ineq:   " << n_ineq   << std::endl;
        LOG_DEBUG_S << "n_bounds: " << n_bounds << std::endl;

        toQpSwift(qp);
        my_qp = QP_SETUP_dense(n_dec,                  // Number decision variables
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

        LOG_DEBUG_S << "Setup Time     : " << my_qp->stats->tsetup * 1000.0 << " ms" << std::endl;
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
        throw std::runtime_error("QPSwiftSolver failed:LDL Factorization failed");
    }
    }

    for(uint i = 0; i < n_dec; i++)
        solver_output[i] = my_qp->x[i];
}

}
