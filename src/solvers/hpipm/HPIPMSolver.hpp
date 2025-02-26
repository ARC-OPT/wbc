#ifndef WBC_SOLVERS_HPIPM_SOLVER_HPP
#define WBC_SOLVERS_HPIPM_SOLVER_HPP

#include "../../core/QPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"
#include <acados_c/dense_qp_interface.h>

namespace wbc {

class HPIPMSolver : public QPSolver{
private:
    static QPSolverRegistry<HPIPMSolver> reg;
    std::vector<int> idxb;
    dense_qp_in *qp_in;
    dense_qp_dims dims;
    void *opts;
    dense_qp_out *qp_out;
    dense_qp_solver *qp_solver;
    qp_solver_config *config;

    std::string returnCodeToString(int code);

public:
    HPIPMSolver();
    virtual ~HPIPMSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output, bool allow_warm_start = true);

    dense_qp_solver_plan plan;

    void setOptions(std::string &field, void* value);

};
}

#endif
