#ifndef HierarchicalDCPISolver_HPP
#define HierarchicalDCPISolver_HPP

#include <Eigen/Core>
#include <vector>
#include "GeneralizedInverse.hpp"

namespace wbc{

class SolverInput;
/**
 * @brief The HierarchicalForceSolver class implements a force-torque based prioritized
 *        redundancy resolution scheme. It uses the inertia weighted Pseudo Inverse to
 *        achieve dynamic decoupling between operational and nullspace dynamics.
 *        Throughout the code, the index y denotes the dependent (task) variables, the index x the independent (joint) variables.
 */
class HierarchicalDCPISolver{

    class Priority
    {
    public:
        Priority(const uint ny, uint nx){
            gi = new GeneralizedInverse(ny, nx);
        }
        ~Priority(){
            delete gi;
        }

        GeneralizedInverse* gi;
    };

public:
    HierarchicalDCPISolver();
    ~HierarchicalDCPISolver();

    bool configure(const std::vector<uint> ny_per_prio, const uint nx);
    void solve(const SolverInput& input, Eigen::VectorXd& x);

protected:
    void clearPriorities();

    uint nx_;      /** No of independent variables used in the problem */
    Eigen::MatrixXd proj_; /** nx*nx Projection operator, that projects the task Jacobians on the Nullspace of the previous priority */
    std::vector<Priority*> priorities_;


}; //class HierarchicalDCPISolver

} //namespace

#endif // HierarchicalDCPISolver_HPP
