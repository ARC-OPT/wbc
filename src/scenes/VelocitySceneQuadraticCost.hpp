#ifndef VELOCITYSCENEQUADRATICCOST_HPP
#define VELOCITYSCENEQUADRATICCOST_HPP

#include "../scenes/VelocityScene.hpp"

namespace wbc{

/**
 * @brief Velocity-based implementation of the WBC Scene. It sets up and solves the following problem:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize & \| \mathbf{J}_w\dot{\mathbf{q}} - \mathbf{v}_d\|_2& \\
 *            \mathbf{\dot{q}} & & \\
 *             & & \\
 *           s.t. & \mathbf{J}_{c,i}\dot{\mathbf{q}}=0, \, \forall i & \\
 *                & \dot{\mathbf{q}}_{m} \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_{M} & \\
 *        \end{array}
 *  \f]
 *
 * In contrast to the VelocityScene class, the tasks are formulated within the cost function instead of modeling them as constraints. The problem is
 * solved with respect to a number of rigid contacts \f$\mathbf{J}_{c,i}\dot{\mathbf{q}}=0, \, \forall i \f$ and under consideration of the joint velocity limits of the robot.
 *
 * \f$\dot{\mathbf{q}}\f$ - Vector of robot joint velocities<br>
 * \f$\mathbf{v}_{d}\f$ - Desired Spatial velocities of all tasks stacked in a vector<br>
 * \f$\mathbf{J}\f$ - Task Jacobians of all tasks stacked in a single matrix<br>
 * \f$\mathbf{J}_w = \mathbf{W}\mathbf{J}\f$ - Weighted task Jacobians<br>
 * \f$\mathbf{W}\f$ - Diagonal task weight matrix<br>
 * \f$\dot{\mathbf{q}}_{m},\dot{\mathbf{q}}_{M}\f$ - Joint velocity limits<br>
 * \f$\mathbf{J}_{c,i}\f$ - Contact Jcaobian of i-th contact point<br>
 *
 *
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
    VelocitySceneQuadraticCost(RobotModelPtr robot_model, QPSolverPtr solver, double dt=0.001);
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
