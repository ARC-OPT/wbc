#ifndef OPTPROBLEM_HPP
#define OPTPROBLEM_HPP

#include <base/Eigen.hpp>
#include <vector>

namespace wbc{

/**
 * @brief Generic base class for optimization problems
 */
class OptProblem{
public:
};

/**
 * @brief Describes a Weighte least squares optimization problem
 */
class WeightedLS{
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

/**
 * @brief Describes a hierarchical weighted least squares optimization problem
 */
class HierarchicalWeightedLS : public OptProblem{
public:
    std::vector<WeightedLS> prios;
};

}

#endif // OPTPROBLEM_HPP
