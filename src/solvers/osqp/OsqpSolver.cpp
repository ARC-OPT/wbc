#include "OsqpSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <chrono>

namespace wbc {

QPSolverRegistry<OsqpSolver> OsqpSolver::reg("osqp");

OsqpSolver::OsqpSolver() : configured(false){
}

OsqpSolver::~OsqpSolver(){
}

void OsqpSolver::resize(uint nq, uint nc){
    solver.data()->setNumberOfConstraints(nc);
    solver.data()->clearLinearConstraintsMatrix();
    solver.data()->clearHessianMatrix();

    hessian_sparse.resize(nq,nq);
    gradient.resize(nq);
    constraint_mat_dense.resize(nc,nq);
    constraint_mat_sparse.resize(nc,nq);
    lower_bound.resize(nc);
    upper_bound.resize(nc);

    constraint_mat_dense.setZero();
    lower_bound.setZero();
    upper_bound.setZero();
    hessian_sparse.setZero();
    gradient.setZero();
    constraint_mat_sparse.setZero();

    solver.data()->setNumberOfVariables(nq);
    solver.data()->setNumberOfConstraints(nc);
    solver.data()->setHessianMatrix(hessian_sparse);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(constraint_mat_sparse);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);
    solver.settings()->setVerbosity(false);
}

void OsqpSolver::solve(const HierarchicalQP& hierarchical_qp, Eigen::VectorXd& solver_output, bool allow_warm_start){

    assert(hierarchical_qp.size() == 1);

    const QuadraticProgram& qp = hierarchical_qp[0];
    assert(qp.isValid());

    uint nc = qp.bounded ? qp.neq + qp.nin + qp.nq : qp.neq + qp.nin;

    if(!allow_warm_start)
        configured = false;

    if(!configured){
        resize(qp.nq, nc);
        configured = true;
        solver.initSolver();
    }

    if(solver.data()->getData()->m != nc)
        resize(qp.nq, nc);

    // OSQP treats bounds, eq. and ineq. constraints in one constraint matrix / vector, so we have to merge:
    if(qp.neq != 0){      // Has equality constraints
        constraint_mat_dense.middleRows(0,qp.neq) = qp.A;
        lower_bound.segment(0,qp.neq) = qp.b;
        upper_bound.segment(0,qp.neq) = qp.b;
    }
    if(qp.C.rows() != 0){ // Has inequality constraints
        constraint_mat_dense.middleRows(qp.neq,qp.nin) = qp.C;
        lower_bound.segment(qp.neq,qp.nin) = qp.lower_y;
        upper_bound.segment(qp.neq,qp.nin) = qp.upper_y;
    }
    if(qp.bounded){       // Has bounds
        constraint_mat_dense.middleRows(qp.neq+qp.nin,qp.nq).diagonal().setConstant(1.0);
        lower_bound.segment(qp.neq+qp.nin,qp.nq) = qp.lower_x;
        upper_bound.segment(qp.neq+qp.nin,qp.nq) = qp.upper_x;
    }

    constraint_mat_sparse = constraint_mat_dense.sparseView();
    hessian_dense = qp.H;
    hessian_sparse = hessian_dense.sparseView();
    gradient = qp.g;

    solver.updateHessianMatrix(hessian_sparse);
    solver.updateLinearConstraintsMatrix(constraint_mat_sparse);
    solver.updateGradient(gradient);
    solver.updateBounds(lower_bound,upper_bound);


    OsqpEigen::ErrorExitFlag flag = solver.solveProblem();
    if(flag != OsqpEigen::ErrorExitFlag::NoError){
        qp.print();
        throw std::runtime_error("Error solving QP: " + exitFlagToString(flag));
    }
    solver_output = solver.getSolution();

}

}
