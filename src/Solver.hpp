#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <vector>
#include <base/Eigen.hpp>

namespace wbc{

class OptProblem;

class Solver{
public:
    Solver(){}
    virtual ~Solver(){}

    /**
     * @brief configure Interface for configuration of the solver. Implement in your concrete solver implementation. This should
     *        prepare all data structures.
     * @param n_constraint_per_prio Number of constraints per priority
     * @param n_joints Number of robot joints
     * @return true in case of success, false in case of failure.
     */
    virtual bool configure(const std::vector<int>& n_constraint_per_prio, const unsigned int n_joints) = 0;

    /**
     * @brief solve Solve the given Linear equation system. Throw in case of an error
     * @param linear_eqn_pp
     * @param solver_output
     */
    virtual void solve(const OptProblem &opt_problem, base::VectorXd &solver_output) = 0;

    /**
     * @brief Sets the weight for each individual joint.
     * @param weights Vector size has to be same as number of joints (see configure()). Weight values have to be >= 0.
     * A zero weights means that the joint is not considered for the solution.
     */
    virtual void setJointWeights(const base::VectorXd& weights) = 0;
};
}

#endif // SOLVER_HPP
