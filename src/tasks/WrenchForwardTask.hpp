#ifndef WBC_TASKS_WRENCHFORWARDTASK_HPP
#define WBC_TASKS_WRENCHFORWARDTASK_HPP

#include "../core/Task.hpp"

namespace wbc{

class WrenchForwardTask : public Task{
protected:
    Eigen::MatrixXd ref_frame_rotation;
    const std::string ref_frame;
public:
    WrenchForwardTask(TaskConfig config,
                      RobotModelPtr robot_model,
                      const std::string &ref_frame);
    virtual ~WrenchForwardTask() = default;

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

    /** @brief Returns the reference frame. This is the frame in which the task reference is assumed to be given. In this case the given reference wrench
     *  will be transformed from this frame to world frame*/
    const std::string& refFrame(){return ref_frame;}
};

typedef std::shared_ptr<WrenchForwardTask> WrenchForwardTaskPtr;

}

#endif // CARTESIANWRENCHTASK_HPP
