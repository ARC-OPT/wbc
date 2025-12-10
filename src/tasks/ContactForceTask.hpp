#ifndef WBC_TASKS_CONTACTFORCETASK_HPP
#define WBC_TASKS_CONTACTFORCETASK_HPP

#include "../core/Task.hpp"

namespace wbc{

class ContactForceTask : public Task{
public:
    ContactForceTask(TaskConfig config,
                      RobotModelPtr robot_model);
    virtual ~ContactForceTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update() override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the wrench part is relevant (Must have a valid force and torque!)
     */
    void setReference(const types::Wrench& ref);
};

typedef std::shared_ptr<ContactForceTask> ContactForceTaskPtr;

}

#endif // CARTESIANWRENCHTASK_HPP
