#ifndef WBC_SOLVERS_SOLVER_HPP
#define WBC_SOLVERS_SOLVER_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <memory>

namespace wbc{

class HierarchicalQP;

class QPSolver{
protected:
    bool configured;
public:
    QPSolver() : configured(false){}
    virtual ~QPSolver(){}
    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const HierarchicalQP& hierarchical_qp, base::VectorXd &solver_output) = 0;

    /** @brief reset Enforces reconfiguration at next call to solve() */
    void reset(){configured=false;}
};

typedef std::shared_ptr<QPSolver> QPSolverPtr;

}

#endif
