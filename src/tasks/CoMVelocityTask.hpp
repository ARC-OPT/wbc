#ifndef WBC_TASKS_COMVELOCITYTASK_HPP
#define WBC_TASKS_COMVELOCITYTASK_HPP

#include "../core/Task.hpp"

namespace wbc{

/**
 * @brief Implementation of a CoM velocity task.
 */
class CoMVelocityTask : public Task{
public:
    CoMVelocityTask(TaskConfig config,
                    RobotModelPtr robot_model);
    virtual ~CoMVelocityTask() = default;

    virtual void update() override;

    /**
     * @brief Update the CoM reference input for this task.
     * @param ref Reference input for this task. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    void setReference(const Eigen::Vector3d& ref);
};

typedef std::shared_ptr<CoMVelocityTask> CoMVelocityTaskPtr;

} // namespace wbc

#endif
