#ifndef COM_ACCELERATION_TASK_HPP
#define COM_ACCELERATION_TASK_HPP

#include "CartesianTask.hpp"

namespace wbc{

/**
 * @brief Implementation of a CoM velocity task.
 */
class CoMAccelerationTask : public CartesianTask{
public:
    CoMAccelerationTask(TaskConfig config, uint n_robot_joints);
    virtual ~CoMAccelerationTask() = default;

    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the CoM reference input for this task.
     * @param ref Reference input for this task. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref);
};

typedef std::shared_ptr<CoMAccelerationTask> CoMAccelerationTaskPtr;

} // namespace wbc

#endif
