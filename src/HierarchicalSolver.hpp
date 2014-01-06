#ifndef HIERARHICAL_SOLVER_HPP
#define HIERARHICAL_SOLVER_HPP

#include <vector>
#include <Eigen/Core>

class HierarchicalSolver{

public:
    HierarchicalSolver(){}
    virtual ~HierarchicalSolver(){}

    /**
     * @brief configure Resizes member variables
     * @param ny_per_prio Number of task variables per priority
     * @param nx Number of joint space variables
     * @return True in case of successful initialization, false else
     */
    virtual bool configure(const std::vector<unsigned int>& ny_per_prio, const unsigned int nx) = 0;

    /**
     * @brief solve Compute optimal control solution
     * @param A Linear equation system describing the tasks, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param y Task variables, sorted by priority levels. The first element of the vector corresponds to the highest priority
     * @param x Control solution in joint space
     */
    virtual void solve(const std::vector<Eigen::MatrixXd> &A,
                       const std::vector<Eigen::VectorXd> &y,
                       Eigen::VectorXd &x) = 0;

    virtual void setTaskWeights(const Eigen::VectorXd& weights, const uint prio) = 0;
    virtual void setJointWeights(const Eigen::VectorXd& weights) = 0;
};

#endif
