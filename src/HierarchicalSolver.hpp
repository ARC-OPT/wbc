#ifndef HIERARCHICAL_SOLVER_HPP
#define HIERARCHICAL_SOLVER_HPP

#include <Eigen/Core>
#include <vector>

class HierarchicalSolver{
public:
    HierarchicalSolver(){}
    virtual ~HierarchicalSolver(){}

    /**
     * @brief configure Initialize member variables. Further Configurations will have to be done by the derived class, since they will propably be solver specific
     * @param ny_per_priority No of task variables per priority level.
     * @param nx No of independent configuration space variables
     * @return False in case of an error, true in case of success
     */
    virtual bool configure(const std::vector<unsigned int>& ny_per_priority, const unsigned int nx) = 0;

    /**
     * @brief solve
     * @param A
     * @param y
     * @param solver_output
     * @return
     */
    virtual void solve(const std::vector<Eigen::MatrixXd> &A,
                       const std::vector<Eigen::VectorXd> &y,
                       Eigen::VectorXd &solver_output) = 0;
};

#endif
