#ifndef JOINTACCELERATIONTASK_HPP
#define JOINTACCELERATIONTASK_HPP

#include "JointTask.hpp"

namespace wbc{
/**
 * @brief Implementation of a Joint velocity task.
 */
class JointAccelerationTask : public JointTask{
public:
    JointAccelerationTask(TaskConfig config, uint n_robot_joints);
    virtual ~JointAccelerationTask() = default;

    /**
     * @brief Compute the joint task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the joint task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;


    /**
     * @brief Update the Joint reference input for this task.
     * @param ref Joint reference input. Vector size has ot be same number of task variables. Joint Names have to match the task joint names.
     * Each entry has to have a valid acceleration. All other entries will be ignored.
     */
    virtual void setReference(const base::commands::Joints& ref);
};

} // namespace wbc

#endif
