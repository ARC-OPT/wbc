#ifndef JOINTVELOCITYTASK_HPP
#define JOINTVELOCITYTASK_HPP

#include "JointTask.hpp"

namespace wbc{
/**
 * @brief Implementation of a Joint velocity task.
 */
class JointVelocityTask : public JointTask{
public:
    JointVelocityTask(TaskConfig config, uint n_robot_joints);
    virtual ~JointVelocityTask() = default;

    /**
     * @brief Compute the joint task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the joint task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the Joint reference input for this task.
     * @param ref Joint reference input. Vector size has ot be same number of task variables. Joint Names have to match the task joint names.
     * Each entry has to have a valid velocity. All other entries will be ignored.
     */
    virtual void setReference(const base::commands::Joints& ref);
};

typedef std::shared_ptr<JointVelocityTask> JointVelocityTaskPtr;

} // namespace wbc

#endif
