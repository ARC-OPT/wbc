#ifndef WBC_TYPES_QUADRATIC_PROGRAM_HPP
#define WBC_TYPES_QUADRATIC_PROGRAM_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

class JointWeights : public base::NamedVector<double>{
};

/**
 * @brief Describes a quadratic program of the form
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{x}) & \frac{1}{2} \mathbf{x}^T\mathbf{H}\mathbf{x}+\mathbf{x}^T\mathbf{g}& \\
 *             & & \\
 *        s.t. & lb(\mathbf{Ax}) \leq \mathbf{Ax} \leq ub(\mathbf{Ax})& \\
 *             & lb(\mathbf{x}) \leq \mathbf{x} \leq ub(\mathbf{x})& \\
 *        \end{array}
 *  \f]
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
    int nc;                 /** Number of constraints for this prio*/
    int nq;                 /** Number of all joints (actuated + unactuated)*/

    /** Initialize all variables with NaN */
    void resize(const uint nc, const uint nq);

};

/**
 * @brief Describes a hierarchy of quadratic programs
 */
struct HierarchicalQP{
    base::Time time;
    std::vector<QuadraticProgram> prios;           /** Hierarchical organized QPs. The first entriy is the highest priority*/
    base::VectorXd Wq;                             /** Joint weights (all joints) */

    size_t size() const {
        return prios.size();
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
