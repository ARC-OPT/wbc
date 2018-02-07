#ifndef LINEAR_EQUALITY_CONSTRAINTS_HPP
#define LINEAR_EQUALITY_CONSTRAINTS_HPP

#include <base/Eigen.hpp>

namespace wbc{

/**
 * @brief Describes a set of linear constraints Ax=y with optional weighting for an optimization problem
 */
class LinearEqualityConstraints{
public:
    base::MatrixXd A;     /** Constraint matrix */
    base::VectorXd y_ref; /** Desired solution*/
    base::VectorXd W;     /** weights */

    void resize(const uint rows, const uint cols)
    {
        A.resize(rows, cols);
        y_ref.resize(rows);
        W.resize(rows);
    }
};
}

#endif // LinearEqualityConstraints
