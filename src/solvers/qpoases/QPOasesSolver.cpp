#include "QPOasesSolver.hpp"
#include "../../types/QuadraticProgram.hpp"
#include <base/Eigen.hpp>
#include <Eigen/Core>
#include <iostream>

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
        std::cout<<"NWSR IS "<<actual_n_wsr<<std::endl;
        ret_val = sq_problem.init(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN){
            std::cout<<"SOLVER INPUT: "<<std::endl;
            std::cout<<"H"<<std::endl;
            std::cout<<qp.H<<std::endl;
            std::cout<<"g"<<std::endl;
            std::cout<<qp.g.transpose()<<std::endl;
            std::cout<<"A"<<std::endl;
            std::cout<<qp.A<<std::endl;
            std::cout<<"lower_y"<<std::endl;
            std::cout<<qp.lower_y.transpose()<<std::endl;
            std::cout<<"upper_y"<<std::endl;
            std::cout<<qp.upper_y.transpose()<<std::endl;
            std::cout<<"lower_x"<<std::endl;
            std::cout<<qp.lower_x.transpose()<<std::endl;
            std::cout<<"upper_x"<<std::endl;
            std::cout<<qp.upper_x.transpose()<<std::endl<<std::endl;
            throw std::runtime_error("SQ Problem initialization failed with error " + std::to_string(ret_val));
        }
    }
    else{
        ret_val = sq_problem.hotstart(H_ptr, g_ptr, A_ptr, lb_ptr, ub_ptr, lbA_ptr, ubA_ptr, actual_n_wsr, 0);
        if(ret_val != SUCCESSFUL_RETURN){
            std::cout<<"SOLVER INPUT: "<<std::endl;
            std::cout<<"H"<<std::endl;
            std::cout<<qp.H<<std::endl;
            std::cout<<"g"<<std::endl;
            std::cout<<qp.g.transpose()<<std::endl;
            std::cout<<"A"<<std::endl;
            std::cout<<qp.A<<std::endl;
            std::cout<<"lower_y"<<std::endl;
            std::cout<<qp.lower_y.transpose()<<std::endl;
            std::cout<<"upper_y"<<std::endl;
            std::cout<<qp.upper_y.transpose()<<std::endl;
            std::cout<<"lower_x"<<std::endl;
            std::cout<<qp.lower_x.transpose()<<std::endl;
            std::cout<<"upper_x"<<std::endl;
            std::cout<<qp.upper_x.transpose()<<std::endl<<std::endl;
            throw std::runtime_error("SQ Problem hotstart failed with error " + std::to_string(ret_val));
        }
    }

    int nj = hierarchical_qp.nJoints();
    //solver_output.resize(2*hierarchical_qp.nJoints()+6);
    solver_output.resize(2*hierarchical_qp.nJoints());
    if(sq_problem.getPrimalSolution( solver_output.data() ) == RET_QP_NOT_SOLVED)
        throw std::runtime_error("SQ Problem getPrimalSolution() returned " + std::to_string(RET_QP_NOT_SOLVED));

    if((base::Time::now()-stamp).toSeconds()>1){
        stamp = base::Time::now();
        std::cout<<"Solution Acc: "<<solver_output.segment(0,nj).transpose()<<std::endl;
        //std::cout<<"Solution F_ext: "<<solver_output.segment(nj,6).transpose()<<std::endl;
        std::cout<<"Solution Tau: "<<solver_output.segment(nj,nj).transpose()<<std::endl;
        std::cout<<"--------------------------------------------------------------"<<std::endl;
    }

}

returnValue QPOASESSolver::getReturnValue(){
    return ret_val;
}

void QPOASESSolver::setOptions(const qpOASES::Options& opt){
    options = opt;
    sq_problem.setOptions(opt);
}

}
