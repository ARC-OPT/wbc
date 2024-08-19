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

void HPIPMSolver::solve(const HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output){

    assert(hierarchical_qp.size() == 1);
    const QuadraticProgram &qp = hierarchical_qp[0];
    assert(qp.isValid());

    config = dense_qp_config_create(&plan);

    if(!configured || dims.ne != qp.neq || dims.ng != qp.nin){

        dims.nv = qp.nq;
        dims.ne = qp.neq;
        dims.nb = qp.bounded ? qp.nq : 0;
        dims.ng = qp.nin;
        dims.ns = 0;
        dims.nsb = 0;
        dims.nsg = 0;

        qp_in = dense_qp_in_create(config, &dims);
        opts = dense_qp_opts_create(config, &dims);

        dense_qp_hpipm_opts *hpipm_opts = (dense_qp_hpipm_opts *)opts;
        hpipm_opts->hpipm_opts->warm_start = 2;
        hpipm_opts->hpipm_opts->mode = SPEED;

        qp_out = dense_qp_out_create(config, &dims);
        qp_solver = dense_qp_create(config, &dims, opts);

        idxb.resize(qp.nq);
        for(int i = 0; i < qp.nq; i++)
            idxb[i] = i;

        configured = true;
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

    int acados_return = dense_qp_solve(qp_solver, qp_in, qp_out);
    if(acados_return != ACADOS_SUCCESS)
        std::cout<<"Acados returned "<<acados_return<<std::endl;

    solver_output.resize(qp.nq);
    solver_output.setZero();
    d_dense_qp_sol_get_v(qp_out,solver_output.data());
}

}