#ifndef QP_SWIFT_SOLVER_HPP
#define QP_SWIFT_SOLVER_HPP

#include "../../core/QPSolver.hpp"
#include <qpSWIFT/qpSWIFT.h>

namespace wbc {
class QuadraticProgram;

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
    uint max_iter;      /** Maximum number of Iterations of QP */
    double rel_tol;     /** Relative Tolerance */
    double abs_tol;     /** Absolute Tolerance */
    double sigma;       /** sigma desired */
    uint verbose_level; /** Verbose Levels, 0 - Print,  >0 - Print Everything */

    void toQpSwift(const QuadraticProgram &qp);
public:
    QPSwiftSolver();
    ~QPSwiftSolver();

    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const wbc::HierarchicalQP &hierarchical_qp, Eigen::VectorXd &solver_output);

    void setMaxIter(uint val){max_iter=val;}
    void setRelTol(double val){rel_tol=val;}
    void setAbsTol(double val){abs_tol=val;}
    void setSigma(double val){sigma=val;}
    void setVerboseLevel(uint val){verbose_level=val;}
};
}

#endif
