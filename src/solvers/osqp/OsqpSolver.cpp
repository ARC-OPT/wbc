#include "OsqpSolver.hpp"
#include "../../core/QuadraticProgram.hpp"

namespace wbc {

OsqpSolver::OsqpSolver() : configured(false){
}

OsqpSolver::~OsqpSolver(){
}

void OsqpSolver::solve(const HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output){

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("OsqpSolver::solve: Constraints vector size must be 1 for the current implementation");

    const QuadraticProgram& qp = hierarchical_qp[0];
    uint nc = qp.bounded ? qp.neq + qp.nin + qp.nq : qp.neq + qp.nin;

    if(!configured){
        hessian_sparse.resize(qp.nq,qp.nq);
        gradient.resize(qp.nq);
        constraint_mat_dense.resize(nc,qp.nq);
        constraint_mat_dense.setZero();
        constraint_mat_sparse.resize(nc,qp.nq);
        lower_bound.resize(nc);
        upper_bound.resize(nc);
        lower_bound.setZero();
        upper_bound.setZero();
        hessian_sparse.setZero();
        gradient.setZero();
        constraint_mat_sparse.setZero();
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(qp.nq);
        configured = true;
        solver.data()->setNumberOfConstraints(nc);
        solver.data()->setHessianMatrix(hessian_sparse);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(constraint_mat_sparse);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
        solver.initSolver();
    }


    if(solver.data()->getData()->m != nc){
        solver.data()->setNumberOfConstraints(nc);
        solver.data()->clearLinearConstraintsMatrix();
        solver.data()->clearHessianMatrix();
        hessian_sparse.resize(qp.nq,qp.nq);
        gradient.resize(qp.nq);
        constraint_mat_dense.resize(nc,qp.nq);
        constraint_mat_dense.setZero();
        constraint_mat_sparse.resize(nc,qp.nq);
        lower_bound.resize(nc);
        upper_bound.resize(nc);
        lower_bound.setZero();
        upper_bound.setZero();
        hessian_sparse.setZero();
        gradient.setZero();
        constraint_mat_sparse.setZero();
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(qp.nq);
        solver.data()->setNumberOfConstraints(nc);
        solver.data()->setHessianMatrix(hessian_sparse);
        solver.data()->setGradient(gradient);
        solver.data()->setLinearConstraintsMatrix(constraint_mat_sparse);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
    }

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
