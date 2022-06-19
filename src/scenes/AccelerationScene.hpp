#ifndef ACCELERATIONSCENE_HPP
#define ACCELERATIONSCENE_HPP

#include "../core/Scene.hpp"

namespace wbc{


/**
 * @brief Acceleration-based implementation of the WBC Scene. It sets up and solves the following optimization problem:
 *
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}_w\ddot{\mathbf{q}} - \dot{\mathbf{v}}_d + \dot{\mathbf{J}}\dot{\mathbf{q}}\|_2\\
 *        \mathbf{\ddot{q}} & & \\
 *        \end{array}
 *  \f]
 * The tasks are all formulated within the cost function, i.e., the problem is unconstrained. Soft prioritization can be achieved using task weights, as \f$\mathbf{J}_w=\mathbf{W}\mathbf{J}\f$.
 *
 * \f$\ddot{\mathbf{q}}\f$ - Vector of robot joint accelerations<br>
 * \f$\dot{\mathbf{v}}_{d}\f$ - Desired spatial accelerations of all tasks stacked in a vector<br>
 * \f$\mathbf{J}\f$ - Task Jacobians of all tasks stacked in a single matrix<br>
 * \f$\mathbf{J}_w\f$ - Weighted task Jacobians<br>
 * \f$\mathbf{W}\f$ - Diagonal task weight matrix<br>
 * \f$\dot{\mathbf{J}}\dot{\mathbf{q}}\f$ - Acceleration bias<br>
 */
class AccelerationScene : public WbcScene{
protected:
    base::VectorXd solver_output, robot_acc;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual ConstraintPtr createConstraint(const ConstraintConfig &config);

    base::Time stamp;

public:
    AccelerationScene(RobotModelPtr robot_model, QPSolverPtr solver) :
        WbcScene(robot_model, solver){}
    virtual ~AccelerationScene(){
    }
    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @param ctrl_output Control solution that fulfill the given constraints as good as possible
     */
    virtual const HierarchicalQP& update();

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint acceleration command
     */
    virtual const base::commands::Joints& solve(const HierarchicalQP& hqp);

    /**
     * @brief evaluateConstraints Evaluate the fulfillment of the constraints given the current robot state and the solver output
     */
    virtual const ConstraintsStatus &updateConstraintsStatus();
};

} // namespace wbc

#endif
