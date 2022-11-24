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
    qp.check();

    size_t nc = qp.C.rows() + qp.A.rows();

    if(!configured){
        sq_problem = SQProblem(qp.A.cols(), nc);
        sq_problem.setOptions(options);
        configured = true;
    }

    // Have to convert to row-major order matrices. Eigen uses column major by default and
    // qpoases expects the data to be arranged in row-major
    H = qp.H;

    // same for A mtx but equalities and inequalities constraints have to be merged in a single matrix
    A.resize(nc, qp.A.cols());
    A.topRows(qp.A.rows()) = qp.A;
    A.bottomRows(qp.C.rows()) = qp.C;

    // create constraints vectors (merging equalities and inequalities vecs)
    Eigen::VectorXd lower_a(nc);
    Eigen::VectorXd upper_a(nc);
    lower_a << qp.b, qp.lower_y;
    upper_a << qp.b, qp.upper_y;

    // Joint space upper and lower bounds
    real_t* lb_ptr = 0;
    real_t* ub_ptr = 0;
    if(qp.lower_x.size() > 0)
        lb_ptr = (real_t*)qp.lower_x.data();
    if(qp.upper_x.size() > 0)
        ub_ptr = (real_t*)qp.upper_x.data();

    // Constraint space upper and lower bounds
    real_t* lbA_ptr = 0;
    real_t* ubA_ptr = 0;
    if(lower_a.size() > 0)
        lbA_ptr = (real_t*)lower_a.data();
    if(upper_a.size() > 0)
         ubA_ptr = (real_t*)upper_a.data();

    // Constraint matrix
    real_t* A_ptr = A.data();

    // Hessian matrix:
    real_t* H_ptr = (real_t*)H.data();

    // Gradient vector
    real_t* g_ptr = 0;
    if(qp.g.size() > 0)
        g_ptr = (real_t*)qp.g.data();

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
