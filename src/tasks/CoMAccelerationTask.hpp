#ifndef WBC_TASKS_COMACCELERATIONTASK_HPP
#define WBC_TASKS_COMACCELERATIONTASK_HPP

#include "../core/Task.hpp"
#include "../types/SpatialAcceleration.hpp"

namespace wbc{

/**
 * @brief Implementation of a CoM velocity task.
 */
class CoMAccelerationTask : public Task{
public:
    CoMAccelerationTask(TaskConfig config,
                        uint nj);
    virtual ~CoMAccelerationTask() = default;

    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the CoM reference input for this task.
     * @param ref Reference input for this task. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    void setReference(const Eigen::Vector3d& ref);
};

typedef std::shared_ptr<CoMAccelerationTask> CoMAccelerationTaskPtr;

} // namespace wbc

#endif // WBC_TASKS_COM_ACCELERATION_TASK_HPP
