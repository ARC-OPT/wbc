#ifndef LINEAREQUATIONSYSTEM_HPP
#define LINEAREQUATIONSYSTEM_HPP

#include <base/Eigen.hpp>

namespace wbc{

/** Describes a linear eqaution system Ax=y_ref */
struct LinearEquationSystem{

    base::MatrixXd A;     /** System matrix */
    base::VectorXd y_ref; /** Desired solution*/
    base::VectorXd W_row; /** Row weights */
    base::VectorXd W_col; /** Column weights */

    void resize(const uint n_rows, const uint n_cols)
    {
        A.resize(n_rows, n_cols);
        y_ref.resize(n_rows);
        W_row.resize(n_rows);
        W_col.resize(n_cols);
    }
};
}

#endif // LINEAREQUATIONSYSTEM_HPP
