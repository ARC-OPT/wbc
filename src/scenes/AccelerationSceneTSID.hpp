#ifndef WBCACCELERATIONSCENE_HPP
#define WBCACCELERATIONSCENETSID_HPP

#include "../core/Scene.hpp"
#include "../core/JointAccelerationConstraint.hpp"
#include "../core/CartesianAccelerationConstraint.hpp"
#include <base/samples/Wrenches.hpp>

namespace wbc{

typedef std::shared_ptr<CartesianAccelerationConstraint> CartesianAccelerationConstraintPtr;
typedef std::shared_ptr<JointAccelerationConstraint> JointAccelerationConstraintPtr;
/**
 * @brief Acceleration-based implementation of the WBC Scene. It sets up and solves the following problem:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize & \frac{1}{2} \mathbf{\ddot{q}}^T\mathbf{H}\mathbf{\ddot{q}}+\mathbf{\ddot{q}}^T\mathbf{g}& \\
 *             (\mathbf{\ddot{q}},\mathbf{\tau},\mathbf{f})    &  & \\
 *    &  & \\
 *           s.t.  & \mathbf{M}\mathbf{\ddot{q}} - \mathbf{S}^T\mathbf{\tau} - \mathbf{J}^T\mathbf{f} = -\mathbf{h} & \\
 *                 & \mathbf{J}\mathbf{\ddot{q}} = -\dot{\mathbf{J}}\dot{\mathbf{q}}& \\
 *        \end{array}
 *  \f]
 *  \f[
 *        \begin{array}{ccc}
 *         \mathbf{H} & = & \mathbf{J}^T \mathbf{J} \\
 *         \mathbf{g} & = & -(\mathbf{J}^T (\ddot{\mathbf{x}}_{des}-\dot{\mathbf{J}}\dot{\mathbf{q}}))^T \\
 *             & & \\
 *        \end{array}
 *  \f]
 * \f$\ddot{\mathbf{q}}\f$ - Vector of robot joint accelerations<br>
 * \f$\ddot{\mathbf{x}}_{des}\f$ - desired task space accelerations of all tasks stacked in a vector<br>
 * \f$\mathbf{J}\f$ - task Jacobians of all tasks stacked in a single matrix<br>
 * \f$\mathbf{M}\f$ - Joint space inertia matrix<br>
 * \f$\mathbf{S}\f$ - Selection matrix<br>
 * \f$\mathbf{\tau}\f$ - actuation forces/torques<br>
 * \f$\mathbf{h}\f$ - bias forces/torques<br>
 * \f$\mathbf{f}\f$ - external forces
 *
 * The implementation is close to the task-space-inverse dynamics framework (TSID): https://andreadelprete.github.io/teaching/tsid/1_tsid_theory.pdf
 *
 */
class AccelerationSceneTSID : public WbcScene{
protected:
    // Helper variables
    base::VectorXd q_dot;
    base::VectorXd solver_output, robot_acc, solver_output_acc;
    base::samples::Wrenches contact_wrenches;
    base::MatrixXd A, A_weighted;
    base::VectorXd y, wy;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual ConstraintPtr createConstraint(const ConstraintConfig &config);

    base::Time stamp;

public:
    AccelerationSceneTSID(RobotModelPtr robot_model, QPSolverPtr solver) :
        WbcScene(robot_model, solver){}
    virtual ~AccelerationSceneTSID(){
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

    /**
     * @brief Get estimated contact wrenches
     */
    const base::samples::Wrenches& getContactWrenches(){return contact_wrenches;}
};

} // namespace wbc

#endif
