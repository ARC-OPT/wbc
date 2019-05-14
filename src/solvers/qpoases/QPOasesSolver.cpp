#include "QPOasesSolver.hpp"
#include "../../types/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>

using namespace qpOASES;

namespace wbc_solvers{

QPOASESSolver::QPOASESSolver(){
    configured = false;
    n_wsr = 10;
    options.setToDefault();
}

QPOASESSolver::~QPOASESSolver(){

}

void QPOASESSolver::solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output){

    if(hierarchical_qp.size() != 1)
        throw std::runtime_error("QPOASESSolver::solve: Constraints vector size must be 1 for the current implementation");

    const wbc::QuadraticProgram qp = hierarchical_qp[0];

    if(!configured){
        sq_problem = SQProblem(qp.A.cols(), qp.A.rows());
        sq_problem.setOptions(options);
        no_of_joints = qp.A.cols();
        configured = true;
    }

    // Have to convert to row-major order matrices. Eigen uses column major by default and
    // qpoases expects the data to be arranged in row-major
    A = qp.A;
    H = qp.H;

    // Convert to real_t data type
    real_t *lb_ptr = 0;
    real_t *ub_ptr = 0;
    if(qp.lower_x.size() > 0)
        lb_ptr = (real_t*)qp.lower_x.data();
    if(qp.upper_x.size() > 0)
        ub_ptr = (real_t*)qp.upper_x.data();
    real_t *A_ptr = (real_t*)A.data();
    real_t *H_ptr = (real_t*)H.data();
    real_t *g_ptr = (real_t*)qp.g.data();
    real_t* lbA_ptr = (real_t*)qp.lower_y.data();
    real_t* ubA_ptr = (real_t*)qp.upper_y.data();

    actual_n_wsr = n_wsr;
    if(!sq_problem.isInitialised()){
        ret_val = sq_problem.init(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN)
            throw std::runtime_error("SQ Problem initialization failed with error " + std::to_string(ret_val));
    }
    else{
        ret_val = sq_problem.hotstart(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN)
            throw std::runtime_error("SQ Problem hotstart failed with error " + std::to_string(ret_val));
    }

    solver_output.resize(no_of_joints);
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

}
