#ifndef WBC_VELOCITY_SCENE_QP_HPP
#define WBC_VELOCITY_SCENE_QP_HPP

#include "../../core/Scene.hpp"

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
 * In contrast to the VelocityScene class, the tasks are formulated within the cost function instead of modeling them as tasks. The problem is
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
class VelocitySceneQP : public Scene{
protected:
    static SceneRegistry<VelocitySceneQP> reg;

    double hessian_regularizer;
    std::vector< TaskPtr > tasks;
    std::vector< ConstraintPtr > constraints;
    HierarchicalQP hqp;
    bool configured;
    types::JointCommand solver_output_joints;
    Eigen::VectorXd solver_output;

public:
    /**
     * @brief WbcVelocityScene
     * @param robot_model Pointer to the robot model
     * @param solver Solver used to solver the qp optimization problem
     */
    VelocitySceneQP(RobotModelPtr robot_model, QPSolverPtr solver, const double dt);
    virtual ~VelocitySceneQP(){}

    /**
     * @brief Configure the WBC scene. Create tasks and sort them by priority given the task config
     * @param tasks Tasks used in optimization function. Size has to be > 0. All tasks have to be valid. See tasks and TaskConfig.hpp for more details.
     */
    virtual bool configure(const std::vector<TaskPtr> &tasks);

    /**
     * @brief Update the wbc scene
     */
    virtual const HierarchicalQP& update();

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint velocity command
     */
    virtual const types::JointCommand& solve(const HierarchicalQP& hqp);

    /**
     * @brief setHessianRegularizer
     * @param reg This value is added to the diagonal of the Hessian matrix inside the QP to reduce the risk of infeasibility. Default is 1e-8
     */
    void setHessianRegularizer(const double reg){hessian_regularizer=reg;}

    /**
     * @brief Return the current value of hessian regularizer
     */
    double getHessianRegularizer(){return hessian_regularizer;}

    /**
     * @brief Get current solver output in raw values
     */
    const Eigen::VectorXd& getSolverOutputRaw() const { return solver_output; }
};

} // namespace wbc

#endif // WBC_VELOCITY_SCENE_QP_HPP
