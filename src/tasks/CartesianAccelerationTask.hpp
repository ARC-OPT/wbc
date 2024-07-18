#ifndef WBC_TASKS_CARTESIANACCELERATION_TASK_HPP
#define WBC_TASKS_CARTESIANACCELERATION_TASK_HPP

#include "../core/Task.hpp"
#include "../types/SpatialAcceleration.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian acceleration task.
 */
class CartesianAccelerationTask : public Task{
protected:
    Eigen::MatrixXd rot_mat;
    std::string tip_frame;
    std::string ref_frame;
public:
    CartesianAccelerationTask(TaskConfig config,
                              const std::string &tip_frame,
                              const std::string &ref_frame,
                              uint nj);
    virtual ~CartesianAccelerationTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the acceleration part is relevant (Must have a valid linear and angular acceleration!)
     */
    void setReference(const types::SpatialAcceleration& ref);

    /** @brief Returns the tip frame. This is the tip  of kinematic chain used by the task*/
    const std::string &tipFrame(){return tip_frame;}

    /** @brief Returns the reference frame. This is the frame in which the task reference is assumed to be given. In this case the given reference acceleration
     *  will be transformed from this frame to world frame*/
    const std::string& refFrame(){return ref_frame;}
};

using CartesianAccelerationTaskPtr = std::shared_ptr<CartesianAccelerationTask>;

} // namespace wbc

#endif
