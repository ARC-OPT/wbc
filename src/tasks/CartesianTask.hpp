#ifndef CARTESIANTASK_HPP
#define CARTESIANTASK_HPP

#include "../core/Task.hpp"

namespace base{ namespace samples { class RigidBodyStateSE3; } }

namespace wbc {

/**
 * @brief Abstract interface for a task in Cartesian space
 */
class CartesianTask : public Task{
public:
    CartesianTask(const TaskConfig& _config, uint n_robot_joints);
    virtual ~CartesianTask();

    /**
     * @brief Update the Cartesian reference input for this task.
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref) = 0;
};

} //namespace wbc

#endif
