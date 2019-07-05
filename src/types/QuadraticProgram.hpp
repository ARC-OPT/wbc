#ifndef WBC_TYPES_QUADRATIC_PROGRAM_HPP
#define WBC_TYPES_QUADRATIC_PROGRAM_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace wbc{

/**
 * @brief Describes a quadratic program of the form
 *
 *       minimize       x^T * H * x + x^T * g
 *           x
 *       subject to    lower_x  <=  x <= upper_x
 *                     lower_y  <= Ax <= upper_y
 */
class QuadraticProgram{
public:
    base::MatrixXd A;       /** Constraint matrix (nc x nq, where nc = number of constraints, nq = number of joints) */
    base::VectorXd g;       /** Gradient vector (nq x 1) */
    base::VectorXd lower_x; /** Lower bound of the solution vector (nq x 1) */
    base::VectorXd upper_x; /** Upper bound of the solution vector (nq x 1) */
    base::VectorXd lower_y; /** Lower bound of the constraint vector (nc x 1) */
    base::VectorXd upper_y; /** Upper bound of the constraint vector (nc x 1) */
    base::MatrixXd H;       /** Hessian Matrix (nq x nq) */
    base::VectorXd Wy;      /** Constraint weights (nc x 1). Default entry is 1. */

    /** Initialize all variables with NaN */
    void resize(const uint nc, const uint nq);

};

struct HierarchicalQP{
    base::Time time;
    std::vector<std::string> joint_names; /** Vector of joint names (nq x 1)*/
    std::vector<QuadraticProgram> prios;  /** hierarchical organized QPs*/
    base::VectorXd Wq;                    /** Joint weights (nq x 1) */

    size_t size() const {
        return prios.size();
    }
    size_t nJoints() const {
        return joint_names.size();
    }
    QuadraticProgram& operator[](int i) {
        return prios[i];
    }
    const QuadraticProgram& operator[](int i) const {
        return prios[i];
    }
    void operator<<(QuadraticProgram& qp) {
        prios.push_back(qp);
    }
    void resize(const size_t &n){prios.resize(n);}
};

}

#endif // LINEAR_EQUALITY_CONSTRAINTS_HPP