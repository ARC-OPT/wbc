#ifndef WBC_SOLVERS_ACADOS_SOLVER_HPP
#define WBC_SOLVERS_ACADOS_SOLVER_HPP

#include "../../core/QPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <acados_c/dense_qp_interface.h>

namespace wbc {

class AcadosSolver : public QPSolver{
private:
    static QPSolverRegistry<AcadosSolver> reg;
    std::vector<int> idxb;
    dense_qp_in *qp_in;
    dense_qp_dims dims;
    void *opts;
    dense_qp_out *qp_out;
    dense_qp_solver *qp_solver;
    qp_solver_config *config;

public:
    AcadosSolver();
    virtual ~AcadosSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output);

    dense_qp_solver_plan plan;

};
}

#endif
