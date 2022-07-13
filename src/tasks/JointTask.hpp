#ifndef JOINTTASK_HPP
#define JOINTTASK_HPP

#include "../core/Task.hpp"
#include <base/commands/Joints.hpp>

namespace wbc {
/**
 * @brief Abstract interface for a task in joint space
 */
class JointTask : public Task{
public:
    JointTask(const TaskConfig& _config, uint n_robot_joints);
    virtual ~JointTask();

    /**
     * @brief Update the Joint reference input for this task.
     */
    virtual void setReference(const base::commands::Joints& ref) = 0;

};

} //namespace wbc

#endif
