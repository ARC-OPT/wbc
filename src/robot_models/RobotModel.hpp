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
     * @brief Load a model and add it to the overall robot model, e.g. another robot or an object attached to the robot.
     * @param RobotModelConfig Config of the robot model
     */
    virtual bool loadModel(const RobotModelConfig& config) = 0;

    /**
     * @brief Update all task frames with a new joint state.
     * @param joint_state The joint_state vector
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>()) = 0;


    /**
     * @brief getState Return the relative state of two task frames that are defined by source and target frame of the input
     * @param tf_one_name Name of the first task frame
     * @param tf_two_name Name of the second task frame
     * @param state Relative pose (twist and acceleration). E.g. the computed pose will be the second task frame wrt to the first task frame
     */
    virtual void getState(const std::string& tf_one_name,
                          const std::string& tf_two_name,
                          base::samples::RigidBodyState& state) = 0;

    /**
     * @brief getState Return the state of the joints given by joint names
     * @param joint_names Joint names to evaluated
     * @param state Position, (velocity and acceleration) of the given joints
     */
    virtual void getState(const std::vector<std::string> &joint_names,
                          base::samples::Joints& state) = 0;

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
     * @brief Create a task frame that is specific for the implemented model type
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return Pointer to the task frame, or 0 in case of failure
     */
    virtual TaskFrame* createTaskFrame(const std::string &tf_name) = 0;
};
}

#endif // ROBOTMODEL_HPP
