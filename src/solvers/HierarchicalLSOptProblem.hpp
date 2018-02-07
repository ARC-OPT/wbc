#ifndef HIERARCHICAL_LS_OPT_PROBLEM_HPP
#define HIERARCHICAL_LS_OPT_PROBLEM_HPP

#include "core/LinearEqualityConstraints.hpp"
#include "core/OptProblem.hpp"
#include <vector>

namespace wbc{
/**
 * @brief Describes a hierarchical weighted least squares optimization problem
 */
class HierarchicalLSOptProblem : public OptProblem, public std::vector<LinearEqualityConstraints>{
public:
};

}

#endif // HIERARCHICAL_LS_OPT_PROBLEM_HPP
