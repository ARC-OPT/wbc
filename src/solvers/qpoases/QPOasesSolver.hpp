#ifndef WBC_SOLVERS_QP_OASES_SOLVER_HPP
#define WBC_SOLVERS_QP_OASES_SOLVER_HPP

#include "../qp_solver.hpp"
#include <qpOASES.hpp>

namespace wbc {
class HierarchicalQP;
}

namespace wbc_solvers {

class QPOASESSolver : public QPSolver{
public:
    QPOASESSolver();
    ~QPOASESSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param constraints Description of the hierarchical quadratic program to solve. Each vector entry correspond to a stage in the hierarchy where
     *                    the first entry has the highest priority.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output);

    /** Set the maximum number of working set recalculations to be performed during the initial homotopy*/
    void setMaxNoWSR(const uint& n){n_wsr = n;}
    /** Get the maximum number of working set recalculations to be performed during the initial homotopy*/
    uint getMaxNoWSR(){return n_wsr;}
    /** Retrieve the return value from the last QP calculation*/
    qpOASES::returnValue getReturnValue();
    /** Get number of working set recalculations actually performed*/
    int getNoWSR(){return actual_n_wsr;}
    /** Return current solver options*/
    qpOASES::Options getOptions(){return options;}
    /** Set new solver options*/
    void setOptions(const qpOASES::Options& opt);

protected:
    qpOASES::Options options;
    qpOASES::SQProblem sq_problem;
    bool configured;
    int n_wsr, actual_n_wsr;
    qpOASES::returnValue ret_val;
    uint no_of_joints;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A;
};

}

#endif
