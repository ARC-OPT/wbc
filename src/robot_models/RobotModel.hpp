#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include <string>

namespace base{
    namespace samples{
        class Joints;
        class RigidBodyState;
    }
}

namespace wbc{

class TaskFrame;
class RobotModelConfig;

/**
 * @brief Interface for all robot models. Contains all task frames.
 */
class RobotModel{

protected:
    std::vector<TaskFrame*> task_frames;
    std::string base_frame;
    std::vector<std::string> joint_names;

public:
    RobotModel(const std::string& _base_frame = "");
    virtual ~RobotModel();

    /**
     * @brief Load a model and add it to the overall robot model, e.g. another robot or an object attached to the robot
     * @param model_file Filename of the model
     * @param initial_pose Initial position and orientation of the model with respect to the hook frame
     * @param hook_name Frame to which the model shall be attached
     */
    virtual bool loadModel(const RobotModelConfig& config) = 0;

    /**
     * @brief Update all task frames with a new joint state.
     * @param joint_state The joint_state vector
     */
    virtual void update(const base::samples::Joints& joint_state) = 0;

    /**
     * @brief Update the pose of a link (segment) in the model. This feature can be used e.g. if there are multiple robots (to update the relative
     *        pose between them) or if there is an object in the scene (e.g. to update its pose wrt to the camera frame)
     * @param new_pose The new link pose. The SourceFrame has to be the same as the segments name that is to be updated.
     */
    virtual void update(const base::samples::RigidBodyState &new_pose) = 0;

    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return True in case of success, false otherwise (e.g. if the task frame already exists)
     */
    bool addTaskFrame(const std::string& tf_name );

    /**
     * @brief Add multiple Task Frames
     */
    bool addTaskFrames(const std::vector<std::string> &tf_names);

    /**
     * @brief Remove the task frame with the given name. Will do nothing if the task frame does not exist
     */
    void removeTaskFrame(const std::string &tf_name);

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

    /**
     * @brief getJointNames Return all joint names relevant for the configured task frames
     */
    std::vector<std::string> getJointNames(){return joint_names;}

protected:
    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return True in case of success, false otherwise (e.g. if the task frame already exists)
     */
    virtual bool addTaskFrameInternal(const std::string& tf_name) = 0;
};
}

#endif // ROBOTMODEL_HPP
