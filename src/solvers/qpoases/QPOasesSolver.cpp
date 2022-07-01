#include "QPOasesSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>

using namespace qpOASES;

namespace wbc{

QPSolverRegistry<QPOASESSolver> QPOASESSolver::reg("qpoases");

QPOASESSolver::QPOASESSolver(){
    n_wsr = 1000;
    options.setToFast();
    options.printLevel = PL_NONE;
}

QPOASESSolver::~QPOASESSolver(){

}

void QPOASESSolver::solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output){

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("QPOASESSolver::solve: Number of task hierarchies must be 1 for the current implementation");

    const wbc::QuadraticProgram &qp = hierarchical_qp[0];

    if(!configured){
        sq_problem = SQProblem(qp.A.cols(), qp.A.rows());
        sq_problem.setOptions(options);
        configured = true;
    }

    // Have to convert to row-major order matrices. Eigen uses column major by default and
    // qpoases expects the data to be arranged in row-major
    A = qp.A;
    H = qp.H;

    // Joint space upper and lower bounds
    real_t *lb_ptr = 0;
    real_t *ub_ptr = 0;
    if(qp.lower_x.size() > 0){
        if(qp.lower_x.size() != qp.nq)
            throw std::runtime_error("Number of joints in quadratic program is " + std::to_string(qp.nq)
                                     + ", but lower bound has size " + std::to_string(qp.lower_x.size()));

        lb_ptr = (real_t*)qp.lower_x.data();
    }
    if(qp.upper_x.size() > 0){
        if(qp.upper_x.size() != qp.nq)
            throw std::runtime_error("Number of joints in quadratic program is " + std::to_string(qp.nq)
                                     + ", but lower bound has size " + std::to_string(qp.upper_x.size()));
        ub_ptr = (real_t*)qp.upper_x.data();
    }

    // Constraint space upper and lower bounds
    real_t *lbA_ptr = 0;
    real_t *ubA_ptr = 0;
    if(qp.lower_y.size() > 0){
        if(qp.lower_y.size() != qp.nc)
            throw std::runtime_error("Number of constraints in quadratic program is " + std::to_string(qp.nc)
                                     + ", but lower bound has size " + std::to_string(qp.lower_y.size()));
         lbA_ptr = (real_t*)qp.lower_y.data();
    }
    if(qp.upper_y.size() > 0){
        if(qp.upper_y.size() != qp.nc)
            throw std::runtime_error("Number of constraints in quadratic program is " + std::to_string(qp.nc)
                                     + ", but lower bound has size " + std::to_string(qp.upper_y.size()));
         ubA_ptr = (real_t*)qp.upper_y.data();
    }

    // Constraint matrix
    if(A.rows() != qp.nc || A.cols() != qp.nq)
        throw std::runtime_error("Constraint matrix A should have size " + std::to_string(qp.nc) + "x" + std::to_string(qp.nq) +
                                 "but has size " +  std::to_string(A.rows()) + "x" + std::to_string(A.cols()));
    real_t *A_ptr = A.data();

    // Hessian matrix:
    if(H.rows() != qp.nq || H.cols() != qp.nq)
        throw std::runtime_error("Hessian matrix H should have size " + std::to_string(qp.nq) + "x" + std::to_string(qp.nq) +
                                 "but has size " +  std::to_string(H.rows()) + "x" + std::to_string(H.cols()));
    real_t *H_ptr = (real_t*)H.data();

    // Gradient vector
    real_t *g_ptr = 0;
    if(qp.g.size() > 0){
        if(qp.g.size() != qp.nq)
            throw std::runtime_error("Gradient vector g should have size " + std::to_string(qp.nq) + "but has size " + std::to_string(qp.g.size()));
        g_ptr = (real_t*)qp.g.data();
    }

    actual_n_wsr = n_wsr;
    if(!sq_problem.isInitialised()){
        ret_val = sq_problem.init(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN){
            options.print();
            qp.print();
            throw std::runtime_error("SQ Problem initialization failed with error " + std::to_string(ret_val));
        }
    }
    else{
        ret_val = sq_problem.hotstart(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN){
            options.print();
            qp.print();
            throw std::runtime_error("SQ Problem hotstart failed with error " + std::to_string(ret_val));
        }
    }

    solver_output.resize(qp.nq);
    if(sq_problem.getPrimalSolution( solver_output.data() ) == RET_QP_NOT_SOLVED)
        throw std::runtime_error("SQ Problem getPrimalSolution() returned " + std::to_string(RET_QP_NOT_SOLVED));
}

returnValue QPOASESSolver::getReturnValue(){
    return ret_val;
}

void QPOASESSolver::setOptions(const qpOASES::Options& opt){
    options = opt;
    sq_problem.setOptions(opt);
}

void QPOASESSolver::setOptionsPreset(const qpOASES::optionPresets& opt){
    switch(opt){
    case qpOASES::qp_default:{
        options.setToDefault();
        break;
    }
    case qpOASES::qp_reliable:{
        options.setToReliable();
        break;
    }
    case qpOASES::qp_fast:{
        options.setToFast();
        break;
    }
    case qpOASES::qp_unset:{
        break;
    }
    default:{
        throw std::runtime_error("QPOASESSolver::setOptionsPreset: Invalid preset: " + std::to_string(opt));
    }
    }
    sq_problem.setOptions(options);
}

}
