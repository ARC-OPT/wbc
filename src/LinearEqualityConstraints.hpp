#ifndef LINEAR_EQUALITY_CONSTRAINTS_HPP
#define LINEAR_EQUALITY_CONSTRAINTS_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace wbc{

/**
 * @brief Describes a set of linear constraints Ax=y with optional weighting in joint and task space
 */
class LinearEqualityConstraints{
public:
    base::MatrixXd A;     /** Constraint matrix (nc x nq, where nc = number of constraints, nq = number of joints) */
    base::VectorXd y_ref; /** Desired solution (nc x 1) */
    base::VectorXd Wy;    /** Task weights (nc x 1) */
    base::VectorXd Wq;    /** Joint Weights (nq x 1) */

    void resize(const uint nc, const uint nq)
    {
        A.resize(nc, nq);
        A.setConstant(std::numeric_limits<double>::quiet_NaN());

        y_ref.resize(nc);
        y_ref.setConstant(std::numeric_limits<double>::quiet_NaN());

        Wy.resize(nc);
        Wy.setConstant(std::numeric_limits<double>::quiet_NaN());

        Wq.resize(nq);
        Wq.setConstant(std::numeric_limits<double>::quiet_NaN());
    }
};
}

#endif // LINEAR_EQUALITY_CONSTRAINTS_HPP
