#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <base/samples/joints.h>
#include <base/samples/RigidBodyState.hpp>
#include "TaskFrame.hpp"

namespace wbc{

/**
 * @brief Interface for all robot models. Contains all task frames.
 */
class RobotModel{

protected:
    std::vector<TaskFrame*> task_frames;

public:
    RobotModel();
    virtual ~RobotModel();

    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return True in case of success, false otherwise
     */
    virtual bool addTaskFrame(const std::string &tf_name) = 0;

    /**
     * @brief Remove the given task frame
     */
    virtual void removeTaskFrame(const std::string &tf_name) = 0;

    /**
     * @brief Add a partial model to the overall robot model, e.g. another robot or an object attached to the robot
     * @param model_file Filename of the model
     * @param initial_pose Initial position and orientation of the model with respect to the hook frame
     * @param hook_name Frame to which the model shall be attached
     */
    virtual bool addModelFromFile(const std::string& model_file,
                                  base::samples::RigidBodyState& initial_pose,
                                  const std::string hook_name = "") = 0;

    /**
     * @brief Convenience method to add multiple task frames at once
     */
    bool addTaskFrames(const std::vector<std::string> &task_frame_ids);

    /**
     * @brief Update all joints in the model with a new joint state.
     * @param joint_state The joint_state vector
     */
    void update(const base::samples::Joints& joint_state);

    /**
     * @brief Update the pose of a link (segment) in the model. This feature can be used e.g. if there are multiple robots (to update the relative
     *        pose between them) or if there is an object in the scene (e.g. to update its pose wrt to the camera frame)
     * @param new_pose The new link pose. The SourceFrame has to be the same as the segments name that is to be updated.
     */
    void update(const base::samples::RigidBodyState &new_pose);

    /**
     * @brief Return a task frame with the given name. Will throw in case the task frame id does not exist
     */
    TaskFrame* getTaskFrame(const std::string& name);

    /**
     * @brief Return all task frames
     */
    std::vector<TaskFrame*> getTaskFrames(){return task_frames;}

    /**
     * @brief Check if the given task frame already exists in the kinematic model
     */
    bool hasTaskFrame(const std::string &name);
};
}

#endif // ROBOTMODEL_HPP
