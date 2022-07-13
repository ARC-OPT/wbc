#ifndef CARTESIANVELOCITYTASK_HPP
#define CARTESIANVELOCITYTASK_HPP

#include "CartesianTask.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian velocity task.
 */
class CartesianVelocityTask : public CartesianTask{
public:
    CartesianVelocityTask(TaskConfig config, uint n_robot_joints);
    virtual ~CartesianVelocityTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref);
};

typedef std::shared_ptr<CartesianVelocityTask> CartesianVelocityTaskPtr;

} // namespace wbc

#endif
