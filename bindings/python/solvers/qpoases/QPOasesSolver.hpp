#ifndef WBC_PY_QPOASES_SOLVER_HPP
#define WBC_PY_QPOASES_SOLVER_HPP

#include "solvers/qpoases/QPOasesSolver.hpp"

namespace wbc_py {


class QPOASESSolver : public wbc::QPOASESSolver{
public:
    base::VectorXd solve(const wbc::HierarchicalQP &hqp);
    int getReturnValueAsInt();
};
}

#endif
