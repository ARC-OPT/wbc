#ifndef WBC_TASKS_CARTESIANACCELERATION_TASK_HPP
#define WBC_TASKS_CARTESIANACCELERATION_TASK_HPP

#include "../core/Task.hpp"
#include "../types/SpatialAcceleration.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian acceleration task.
 */
class SpatialAccelerationTask : public Task{
protected:
    Eigen::MatrixXd rot_mat;
    std::string tip_frame;
    
public:
    SpatialAccelerationTask(TaskConfig config,
                              RobotModelPtr robot_model,
                              const std::string &tip_frame);
    virtual ~SpatialAccelerationTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update() override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the acceleration part is relevant (Must have a valid linear and angular acceleration!)
     */
    void setReference(const types::SpatialAcceleration& ref);

    /** @brief Returns the tip frame. This is the tip of kinematic chain used by the task*/
    const std::string &tipFrame(){return tip_frame;}
};

using SpatialAccelerationTaskPtr = std::shared_ptr<SpatialAccelerationTask>;

} // namespace wbc

#endif
