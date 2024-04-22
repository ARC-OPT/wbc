#ifndef WRENCHFORWARDTASK_HPP
#define WRENCHFORWARDTASK_HPP

#include "CartesianTask.hpp"

namespace wbc{

class WrenchForwardTask : public CartesianTask{
protected:
    base::Wrench ref_wrench;
    base::Pose ref_frame_pose;
    base::MatrixXd ref_frame_rotation;

public:
    WrenchForwardTask(TaskConfig config, uint n_robot_joints);
    virtual ~WrenchForwardTask() = default;

    /**
     * @brief Compute the cartesian task matrix A
     * @param robot_model Pointer to the robot model from which get the state and compute the cartesian task matrix A
     */
    virtual void update(RobotModelPtr robot_model) override;

    /**
     * @brief Update the Cartesian reference input for this task.
     * @param ref Reference input for this task. Only the wrench part is relevant (Must have a valid force and torque!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref) override;
};

}

#endif // CARTESIANWRENCHTASK_HPP
