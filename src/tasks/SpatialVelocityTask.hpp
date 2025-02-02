#ifndef WBC_TASKS_SPATIALVELOCITYTASK_HPP
#define WBC_TASKS_SPATIALVELOCITYTASK_HPP

#include "../core/Task.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian velocity task.
 */
class SpatialVelocityTask : public Task{
protected:
    Eigen::MatrixXd rot_mat;
    std::string tip_frame;
    std::string ref_frame;
public:
    SpatialVelocityTask(TaskConfig config,
                          RobotModelPtr robot_model,
                          const std::string &tip_frame,
                          const std::string &ref_frame);
    virtual ~SpatialVelocityTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update() override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    void setReference(const types::Twist& ref);

    /** @brief Returns the tip frame. This is the tip  of kinematic chain used by the task*/
    const std::string &tipFrame(){return tip_frame;}

    /** @brief Returns the reference frame. This is the frame in which the task reference is assumed to be given. In this case the given reference twist
     *  will be transformed from this frame to world frame*/
    const std::string& refFrame(){return ref_frame;}
};

typedef std::shared_ptr<SpatialVelocityTask> SpatialVelocityTaskPtr;

} // namespace wbc

#endif
