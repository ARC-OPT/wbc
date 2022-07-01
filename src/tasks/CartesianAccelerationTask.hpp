#ifndef CARTESIANACCELERATION_TASK_HPP
#define CARTESIANACCELERATION_TASK_HPP

#include "CartesianTask.hpp"
#include <base/Acceleration.hpp>

namespace wbc{

base::Vector6d operator+(base::Vector6d a, base::Acceleration b);
base::Vector6d operator-(base::Vector6d a, base::Acceleration b);

/**
 * @brief Implementation of a Cartesian acceleration task.
 */
class CartesianAccelerationTask : public CartesianTask{
public:
    CartesianAccelerationTask(TaskConfig config, uint n_robot_joints);
    virtual ~CartesianAccelerationTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the acceleration part is relevant (Must have a valid linear and angular acceleration!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref);
};

} // namespace wbc

#endif
