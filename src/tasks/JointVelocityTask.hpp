#ifndef WBC_TASKS_JOINTVELOCITYTASK_HPP
#define WBC_TASKS_JOINTVELOCITYTASK_HPP

#include "../core/Task.hpp"

namespace wbc{
/**
 * @brief Implementation of a Joint velocity task.
 */
class JointVelocityTask : public Task{
protected:
    std::vector<std::string> joint_names;
public:
    JointVelocityTask(TaskConfig config,
                      RobotModelPtr robot_model,
                      const std::vector<std::string>& joint_names);
    virtual ~JointVelocityTask() = default;

    /**
     * @brief Compute the joint task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the joint task matrix A
     */
    virtual void update() override;

    /**
     * @brief Update the Joint reference input for this task.
     * @param ref Joint reference input. Vector size has ot be same number of task variables. Joint Names have to match the task joint names.
     * Each entry has to have a valid velocity. All other entries will be ignored.
     */
    void setReference(const Eigen::VectorXd& ref);

    /** @brief Returns the vector of joint names used by the task*/
    const std::vector<std::string>& jointNames(){return joint_names;}
};

typedef std::shared_ptr<JointVelocityTask> JointVelocityTaskPtr;

} // namespace wbc

#endif
