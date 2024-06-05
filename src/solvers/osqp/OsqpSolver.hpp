#ifndef WBC_OSQP_SOLVER_HPP
#define WBC_OSQP_SOLVER_HPP

#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>
#include "../../core/QPSolver.hpp"
#include "../../core/QuadraticProgram.hpp"

namespace wbc {

class OsqpSolver : public QPSolver{
public:
    OsqpSolver();
    ~OsqpSolver();

    virtual void solve(const HierarchicalQP& hierarchical_qp, base::VectorXd& solver_output);

protected:
    bool configured;
    OsqpEigen::Solver solver;

    Eigen::MatrixXd hessian_dense;
    Eigen::SparseMatrix<double> hessian_sparse;
    Eigen::MatrixXd constraint_mat_dense;
    Eigen::SparseMatrix<double> constraint_mat_sparse;
    Eigen::VectorXd gradient;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    void resetData(uint nq, uint nc);
    std::string exitFlagToString(OsqpEigen::ErrorExitFlag flag){
        switch(flag){
            case OsqpEigen::ErrorExitFlag::DataValidationError: return "DataValidationError";
            case OsqpEigen::ErrorExitFlag::SettingsValidationError: return "SettingsValidationError";
            case OsqpEigen::ErrorExitFlag::LinsysSolverLoadError: return "LinsysSolverLoadError";
            case OsqpEigen::ErrorExitFlag::LinsysSolverInitError: return "LinsysSolverInitError";
            case OsqpEigen::ErrorExitFlag::NonCvxError: return "NonCvxError";
            case OsqpEigen::ErrorExitFlag::MemAllocError: return "MemAllocError";
            case OsqpEigen::ErrorExitFlag::WorkspaceNotInitError: return "WorkspaceNotInitError";
        default: return "NoError";
        }
    }
};

}

#endif
