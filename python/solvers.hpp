#ifndef WBC_PY_SOLVER_HPP
#define WBC_PY_SOLVER_HPP

#include "solvers/hls/HierarchicalLSSolver.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"

namespace wbc_py {

class HierarchicalLSSolver : public wbc::HierarchicalLSSolver{
public:
    base::VectorXd solve(const wbc::HierarchicalQP &hqp);
};

class QPOASESSolver : public wbc::QPOASESSolver{
public:
    base::VectorXd solve(const wbc::HierarchicalQP &hqp);
    int getReturnValueAsInt();
};
}

#endif
