#ifndef WBC_SOLVERS_SOLVER_HPP
#define WBC_SOLVERS_SOLVER_HPP

#include <vector>
#include <base/Eigen.hpp>

namespace wbc {
class HierarchicalQP;
}

namespace wbc_solvers{

class QPSolver{
public:
    QPSolver(){}
    virtual ~QPSolver(){}
    /**
     * @brief solve Solve the given quadratic program
     * @param constraints Description of the hierarchical quadratic program to solve. Each vector entry correspond to a stage in the hierarchy where
     *                    the first entry has the highest priority.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP& hierarchical_qp, base::VectorXd &solver_output) = 0;
};
}

#endif
