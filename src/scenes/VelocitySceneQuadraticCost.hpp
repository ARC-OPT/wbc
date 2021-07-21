#ifndef VELOCITYSCENEQUADRATICCOST_HPP
#define VELOCITYSCENEQUADRATICCOST_HPP

#include "../scenes/VelocityScene.hpp"

namespace wbc{

/**
 * @brief Velocity-based implementation of the WBC Scene. It sets up and solves the following problem:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize & \frac{1}{2} \mathbf{\dot{q}}^T\mathbf{H}\mathbf{\dot{q}}+\mathbf{\dot{q}}^T\mathbf{g}& \\
 *            \mathbf{\dot{q}} & & \\
 *             & & \\
 *           s.t. & \dot{\mathbf{q}}_{min} \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_{max} & \\
 *        \end{array}
 *  \f]
 *  \f[
 *        \begin{array}{ccc}
 *         \mathbf{H} & = & \mathbf{J}^T \mathbf{J} \\
 *         \mathbf{g} & = & -(\mathbf{J}^T (\dot{\mathbf{x}}_{des}))^T \\
 *             & & \\
 *        \end{array}
 *  \f]
 * \f$\dot{\mathbf{q}}\f$ - Vector of robot joint velocities<br>
 * \f$\dot{\mathbf{x}}_{des}\f$ - desired task space velocities of all tasks stacked in a vector<br>
 * \f$\mathbf{J}\f$ - task Jacobians of all tasks stacked in a single matrix<br>
 *
 * Compared to the VelocityScene class, the tasks are here modeled within the cost function instead of constraints.
 */
class VelocitySceneQuadraticCost : public VelocityScene{
protected:
    base::VectorXd s_vals, tmp;
    base::MatrixXd sing_vect_r, U;
    double hessian_regularizer;

public:
    /**
     * @brief WbcVelocityScene
     * @param robot_model Pointer to the robot model
     * @param model_tasks_as_constraints Model tasks as constraints (true) or as part of the cost function
     */
    VelocitySceneQuadraticCost(RobotModelPtr robot_model, QPSolverPtr solver);
    virtual ~VelocitySceneQuadraticCost();

    /**
     * @brief Update the wbc scene
     */
    virtual const HierarchicalQP& update();

    /**
     * @brief setHessianRegularizer
     * @param reg This value is added to the diagonal of the Hessian matrix inside the QP to reduce the risk of infeasibility. Default is 1e-8
     */
    void setHessianRegularizer(const double reg){hessian_regularizer=reg;}

    /**
     * @brief Return the current value of hessian regularizer
     */
    double getHessianRegularizer(){return hessian_regularizer;}
};

} // namespace wbc

#endif
