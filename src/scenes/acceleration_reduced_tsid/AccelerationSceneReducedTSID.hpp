#ifndef WBCACCELERATIONSCENEREDUCEDTSID_HPP
#define WBCACCELERATIONSCENEREDUCEDTSID_HPP

#include "../../core/Scene.hpp"
#include "../../types/Wrench.hpp"

namespace wbc{

/**
 * @brief Acceleration-based implementation of the WBC Scene. It sets up and solves the following problem:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}_w\ddot{\mathbf{q}} - \dot{\mathbf{v}}_d + \dot{\mathbf{J}}\dot{\mathbf{q}}\|_2\\
 *        \mathbf{\ddot{q}},\mathbf{\tau},\mathbf{f} & & \\
 *           s.t.  & \mathbf{H}\mathbf{\ddot{q}} - \mathbf{S}^T\mathbf{\tau} - \mathbf{J}_c^T\mathbf{f} = -\mathbf{h} & \\
 *                 & \mathbf{J}_{c,i}\mathbf{\ddot{q}} = -\dot{\mathbf{J}}_{c,i}\dot{\mathbf{q}}, \, \forall i& \\
 *                 & \mathbf{\tau}_m \leq \mathbf{\tau} \leq \mathbf{\tau}_M& \\
 *        \end{array}
 *  \f]
 * \f$\ddot{\mathbf{q}}\f$ - Vector of robot joint accelerations<br>
 * \f$\mathbf{v}_{d}\f$ - Desired spatial accelerations of all tasks stacked in a vector<br>
 * \f$\mathbf{J}\f$ - Task Jacobians of all tasks stacked in a single matrix<br>
 * \f$\mathbf{J}_w = \mathbf{W}\mathbf{J}\f$ - Weighted task Jacobians<br>
 * \f$\mathbf{W}\f$ - Diagonal task weight matrix<br>
 * \f$\mathbf{H}\f$ - Joint space inertia matrix<br>
 * \f$\mathbf{S}\f$ - Selection matrix<br>
 * \f$\mathbf{\tau}\f$ - actuation forces/torques<br>
 * \f$\mathbf{h}\f$ - bias forces/torques<br>
 * \f$\mathbf{f}\f$ - external forces<br>
 * \f$\mathbf{J}_{c,i}\f$ - Contact Jacobian of i-th contact point<br>
 * \f$\dot{\mathbf{J}}\dot{\mathbf{q}}\f$ - Acceleration bias<br>
 * \f$\mathbf{\tau}_m,\mathbf{\tau}_M\f$ - Joint force/torque limits<br>
 *
 * The implementation is close to the task-space-inverse dynamics (TSID) method: https://andreadelprete.github.io/teaching/tsid/1_tsid_theory.pdf.
 * It computes the required joint space accelerations \f$\ddot{\mathbf{q}}\f$, torques \f$\mathbf{\tau}\f$ and contact wrenches \f$\mathbf{f}\f$, required to achieve the given task space
 * accelerations \f$\mathbf{v}_{d}\f$ under consideration of the equations of motion (eom), rigid contacts and joint force/torque limits. Note that onyl a single hierarchy level is allowed here,
 * prioritization can be achieved by assigning suitable task weights \f$\mathbf{W}\f$.
 */
class AccelerationSceneReducedTSID : public Scene{
protected:
    static SceneRegistry<AccelerationSceneReducedTSID> reg;

    // Helper variables
    Eigen::VectorXd robot_acc, solver_output_acc;
    std::vector<types::Wrench> contact_wrenches;
    double hessian_regularizer;

public:
    AccelerationSceneReducedTSID(RobotModelPtr robot_model, QPSolverPtr solver, const double dt);
    virtual ~AccelerationSceneReducedTSID(){}

    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @param ctrl_output Control solution that fulfill the given tasks as good as possible
     */
    virtual const HierarchicalQP& update();

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint acceleration command
     */
    virtual const types::JointCommand& solve(const HierarchicalQP& hqp);

    /**
     * @brief Get estimated contact wrenches
     */
    const std::vector<types::Wrench>& getContactWrenches(){return contact_wrenches;}

    /**
     * @brief setHessianRegularizer
     * @param reg This value is added to the diagonal of the Hessian matrix inside the QP to reduce the risk of infeasibility. Default is 1e-8
     */
    void setHessianRegularizer(const double reg){hessian_regularizer=reg;}

    /**
     * @brief Return the current value of hessian regularizer
     */
    double getHessianRegularizer(){return hessian_regularizer;}

    const Eigen::VectorXd& getSolverOutputRaw() const { return solver_output; }
};

} // namespace wbc

#endif
