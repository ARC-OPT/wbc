#include "HPIPMSolver.hpp"
#include "acados/dense_qp/dense_qp_hpipm.h"
#include <iostream>

namespace wbc{

QPSolverRegistry<HPIPMSolver> HPIPMSolver::reg("hpipm");

HPIPMSolver::HPIPMSolver(){
    plan.qp_solver = DENSE_QP_HPIPM;
    qp_in = 0;
    opts = 0;
    qp_out = 0;
    qp_solver = 0;
    config = 0;
}

HPIPMSolver::~HPIPMSolver(){
    if(qp_in)
        free(qp_in);
    if(opts)
        free(opts);
    if(qp_out)
        free(qp_out);
    if(qp_solver)
        free(qp_solver);
    if(config)
        free(config );
}

void HPIPMSolver::solve(const HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output, bool allow_warm_start){

    assert(hierarchical_qp.size() == 1);
    const QuadraticProgram &qp = hierarchical_qp[0];
    assert(qp.isValid());

    if(!allow_warm_start)
        configured = false;

    if(!configured){

        dims.nv = qp.nq;
        dims.ne = qp.neq;
        dims.nb = qp.bounded ? qp.nq : 0;
        dims.ng = qp.nin;
        dims.ns = 0;
        dims.nsb = 0;
        dims.nsg = 0;

        if(config)
            free(config);
        if(qp_in)
            free(qp_in);
        if(opts)
            free(opts);

        config = dense_qp_config_create(&plan);
        qp_in = dense_qp_in_create(config, &dims);
        opts = dense_qp_opts_create(config, &dims);

        dense_qp_hpipm_opts *hpipm_opts = (dense_qp_hpipm_opts *)opts;
        hpipm_opts->hpipm_opts->warm_start = 0;
        hpipm_opts->hpipm_opts->mode = SPEED;

        if(qp_out)
            free(qp_out);
        if(qp_solver)
            free(qp_solver);

        qp_out = dense_qp_out_create(config, &dims);
        qp_solver = dense_qp_create(config, &dims, opts);

        idxb.resize(qp.nq);
        for(int i = 0; i < qp.nq; i++)
            idxb[i] = i;

        configured = true;
    }
    else{
        dense_qp_hpipm_opts *hpipm_opts = (dense_qp_hpipm_opts *)opts;
        hpipm_opts->hpipm_opts->warm_start = 2;
    }

    d_dense_qp_set_all((double* )qp.H.data(),
                       (double* )qp.g.data(),
                       (double* )qp.A.data(),
                       (double* )qp.b.data(),
                       idxb.data(),
                       (double* )qp.lower_x.data(),
                       (double* )qp.upper_x.data(),
                       (double* )qp.C.data(),
                       (double* )qp.lower_y.data(),
                       (double* )qp.upper_y.data(),
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       qp_in);

    int ret = dense_qp_solve(qp_solver, qp_in, qp_out);
    if(ret != ACADOS_SUCCESS)
        throw std::runtime_error("HPIPM returned: " + returnCodeToString(ret));

    solver_output.resize(qp.nq);
    solver_output.setZero();
    d_dense_qp_sol_get_v(qp_out,solver_output.data());
}

std::string HPIPMSolver::returnCodeToString(int code){
    switch(code){
    case SUCCESS: return "Found solution satisfying accuracy tolerance";
    case MAX_ITER: return "Maximum iteration number reached";
    case MIN_STEP: return "Minimum step length reached";
    case NAN_SOL: return "NaN in solution detected";
    case INCONS_EQ: return "unconsistent equality constraints";
    default: return "Unknown error code";
    }
}

}
