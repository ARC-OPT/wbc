#ifndef QP_SWIFT_SOLVER_HPP
#define QP_SWIFT_SOLVER_HPP

#include "../../core/QPSolver.hpp"
#include <qpSWIFT/qpSWIFT.h>

namespace wbc {
class QuadraticProgram;

/**
 * The QPSwiftSolver solves a quadratic problem of type
 *
 * Reference:
 * Pandala, Abhishek Goud and Ding, Yanran and Park, Hae-Won. QpSWIFT: A Real-Time Sparse Quadratic Program
 * Solver for Robotic Applications, IEEE Robotics and Automation Letters, 2019
 *
 * Parameters:
 *  - See Struct settings in Auxiliary.g
 *  - default values are defined in GlobalOptions.h
 */
class QPSwiftSolver : public QPSolver{
private:
    static QPSolverRegistry<QPSwiftSolver> reg;

protected:
    Eigen::MatrixXd P;   /** Cost function Hessian matrix*/
    Eigen::VectorXd c;   /** Cost function gradient*/
    Eigen::MatrixXd A;   /** Equality constraint matrix*/
    Eigen::MatrixXd G;   /** Inequality constraint matrix*/
    Eigen::VectorXd b;   /** Equality constraint vector*/
    Eigen::VectorXd h;   /** Inequality constraint vector*/
    int n_dec;          /** Number decision variables*/
    int n_ineq;         /** Number inequality constraints*/
    int n_eq;           /** Number equality constraints*/
    int n_bounds;       /** Number of lower/upper bounds on the decision variables*/
    QP *my_qp;

    void toQpSwift(const QuadraticProgram &qp);
    void setOptions(settings opt){options=opt;}
public:
    QPSwiftSolver();
    ~QPSwiftSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output);

    settings options;
};
}

#endif
