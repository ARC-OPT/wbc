#ifndef OPTPROBLEM_HPP
#define OPTPROBLEM_HPP

#include <base/Eigen.hpp>

namespace wbc{

class OptProblem{
public:

};

class WeightedLSSimple : public OptProblem{
public:
    base::MatrixXd A;     /** Constraint matrix */
    base::VectorXd y_ref; /** Desired solution*/
    base::VectorXd W;     /** weights */

    void resize(const uint n_rows, const uint n_cols)
    {
        A.resize(n_rows, n_cols);
        y_ref.resize(n_rows);
        W.resize(n_rows);
    }
};
}

#endif // OPTPROBLEM_HPP
