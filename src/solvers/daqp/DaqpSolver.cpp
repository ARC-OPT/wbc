#include "DaqpSolver.hpp"
#include "../../core/QuadraticProgram.hpp"

#include <cassert>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace wbc {

static_assert(std::is_same<c_float, double>::value, "DaqpSolver requires DAQP built in double precision");

QPSolverRegistry<DaqpSolver> DaqpSolver::reg("daqp");

DaqpSolver::DaqpSolver() :
    problem(),
    work(),
    result(),
    settings_changed(false),
    n_var(0), n_bounds(0), n_in(0), n_eq(0){
    daqp_default_settings(&settings);
}

DaqpSolver::~DaqpSolver(){
    freeWorkspace();
}

void DaqpSolver::freeWorkspace(){
    // The settings are owned by this class, prevent DAQP from freeing them
    work.settings = nullptr;
    free_daqp_workspace(&work);
    free_daqp_ldp(&work);
    work = DAQPWorkspace();
}

void DaqpSolver::setSettings(const DAQPSettings& s){
    settings = s;
    settings_changed = true;
}

void DaqpSolver::setMaxNIter(const int n){
    settings.iter_limit = n;
    settings_changed = true;
}

void DaqpSolver::toDaqp(const wbc::QuadraticProgram& qp){
    H = qp.H;
    f = qp.g;

    // Constraint order: [simple bounds; inequalities; equalities].
    // DAQP treats values beyond +-DAQP_INF as unbounded, so clamp infinite bounds
    if(qp.bounded){
        bupper.head(n_bounds) = qp.upper_x.cwiseMin(DAQP_INF);
        blower.head(n_bounds) = qp.lower_x.cwiseMax(-DAQP_INF);
    }
    A.topRows(n_in) = qp.C;
    bupper.segment(n_bounds, n_in) = qp.upper_y.cwiseMin(DAQP_INF);
    blower.segment(n_bounds, n_in) = qp.lower_y.cwiseMax(-DAQP_INF);

    A.bottomRows(n_eq) = qp.A;
    bupper.tail(n_eq) = qp.b;
    blower.tail(n_eq) = qp.b;
}

void DaqpSolver::solve(const wbc::HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start){

    if(hierarchical_qp.size() != 1)
        throw std::invalid_argument("DaqpSolver: Only a single priority level is supported, but the given QP has " +
                                    std::to_string(hierarchical_qp.size()));
    const wbc::QuadraticProgram &qp = hierarchical_qp[0];
    assert(qp.isValid());

    const int nb = qp.bounded ? qp.nq : 0;
    if(qp.nq != n_var || nb != n_bounds || qp.nin != n_in || qp.neq != n_eq)
        configured = false;
    if(!allow_warm_start || settings_changed)
        configured = false;

    if(!configured){
        freeWorkspace();

        n_var = qp.nq;
        n_bounds = nb;
        n_in = qp.nin;
        n_eq = qp.neq;
        const int m = n_bounds + n_in + n_eq;

        H.resize(n_var, n_var);
        f.resize(n_var);
        A.resize(n_in + n_eq, n_var);
        bupper.resize(m);
        blower.resize(m);
        sense.setZero(m);
        sense.tail(n_eq).setConstant(DAQP_ACTIVE + DAQP_IMMUTABLE);
        x.resize(n_var);
        lam.resize(m);

        toDaqp(qp);

        problem = DAQPProblem();
        problem.n = n_var;
        problem.m = m;
        problem.ms = n_bounds;
        problem.H = H.data();
        problem.f = f.data();
        problem.A = A.data();
        problem.bupper = bupper.data();
        problem.blower = blower.data();
        problem.sense = sense.data();

        work.settings = &settings;
        int exitflag = setup_daqp(&problem, &work, nullptr);
        if(exitflag < 0){
            freeWorkspace();
            throw std::runtime_error("DaqpSolver: Setup failed with exit flag " + std::to_string(exitflag));
        }
        settings_changed = false;
        configured = true;
    }
    else{
        // Update the problem data and reuse the working set of the previous solution (warm start)
        toDaqp(qp);
        problem.sense = work.sense;
        int exitflag = daqp_update_ldp(DAQP_UPDATE_Rinv + DAQP_UPDATE_M + DAQP_UPDATE_v + DAQP_UPDATE_d + DAQP_UPDATE_sense, &work, &problem);
        if(exitflag < 0){
            configured = false;
            throw std::runtime_error("DaqpSolver: Problem update failed with exit flag " + std::to_string(exitflag));
        }
    }

    result = DAQPResult();
    result.x = x.data();
    result.lam = lam.data();
    daqp_solve(&result, &work);

    if(result.exitflag < 0){
        // Start from scratch at the next call, the working set is not reliable
        configured = false;
        switch(result.exitflag){
        case DAQP_EXIT_INFEASIBLE:
            throw std::runtime_error("DaqpSolver failed: Problem is infeasible");
        case DAQP_EXIT_CYCLE:
            throw std::runtime_error("DaqpSolver failed: Cycling detected");
        case DAQP_EXIT_UNBOUNDED:
            throw std::runtime_error("DaqpSolver failed: Problem is unbounded");
        case DAQP_EXIT_ITERLIMIT:
            throw std::runtime_error("DaqpSolver failed: Maximum number of iterations reached");
        case DAQP_EXIT_NONCONVEX:
            throw std::runtime_error("DaqpSolver failed: Problem is non-convex");
        case DAQP_EXIT_OVERDETERMINED_INITIAL:
            throw std::runtime_error("DaqpSolver failed: Overdetermined initial working set");
        default:
            throw std::runtime_error("DaqpSolver failed with exit flag " + std::to_string(result.exitflag));
        }
    }

    solver_output = x;
}

}
