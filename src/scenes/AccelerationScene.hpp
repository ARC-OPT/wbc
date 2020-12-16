#ifndef ACCELERATIONSCENE_HPP
#define ACCELERATIONSCENE_HPP

#include "../core/Scene.hpp"
#include "../core/JointAccelerationConstraint.hpp"
#include "../core/CartesianAccelerationConstraint.hpp"

namespace wbc{

typedef std::shared_ptr<CartesianAccelerationConstraint> CartesianAccelerationConstraintPtr;
typedef std::shared_ptr<JointAccelerationConstraint> JointAccelerationConstraintPtr;

/**
 * @brief Acceleration-based implementation of the WBC Scene. It sets up and solves the following problem:
 *  \f[
 *        \begin{array}{ccc}
 *        min(\mathbf{\ddot{q}}) & \frac{1}{2} \mathbf{\ddot{q}}^T\mathbf{H}\mathbf{\ddot{q}}+\mathbf{\ddot{q}}^T\mathbf{g}& \\
 *             & & \\
 *        \end{array}
 *  \f]
 *  \f[
 *        \begin{array}{ccc}
 *         \mathbf{H} & = & \mathbf{J}^T \mathbf{J} \\
 *         \mathbf{g} & = & -(\mathbf{J}^T (\ddot{\mathbf{x}}_{des}-\dot{\mathbf{J}}\dot{\mathbf{q}}))^T \\
 *             & & \\
 *        \end{array}
 *  \f]
 * where \f$\ddot{\mathbf{q}}\f$ is the vector of robot joint accelerations, \f$\ddot{\mathbf{x}}_{des}\f$ the desired task space accelerations of all tasks stacked in a single vector,
 *  \f$\mathbf{J}\f$
 */
class AccelerationScene : public WbcScene{
protected:
    base::VectorXd q_dot;
    base::VectorXd solver_output_acc, robot_acc;

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
    virtual const ConstraintsStatus &updateConstraintsStatus(const base::samples::Joints& solver_output, const base::samples::Joints& joint_state);
};

} // namespace wbc

#endif
