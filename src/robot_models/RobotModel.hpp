#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include <string>
#include "RobotModelConfig.hpp"

namespace base{
    namespace samples{
        class Joints;
        class RigidBodyState;
    }
}

namespace wbc{

class TaskFrame;

/**
 * @brief Interface for all robot models. Contains all task frames.
 */
class RobotModel{

protected:
    std::vector<TaskFrame*> task_frames;
    std::string base_frame;
    std::vector<std::string> joint_names;

public:
    RobotModel();
    virtual ~RobotModel();

    /**
     * @brief configure Configure the robot model.
     * @param robot_model_config Vector of models that will be added to the overall model.
     * @param task_frame_ids IDs of all task frames that are required for the task
     * @return true in case of success, fals otherwise
     */
    virtual bool configure(const std::vector<RobotModelConfig>& _robot_model_config,
                           const std::vector<std::string>& _task_frame_ids,
                           const std::string &_base_frame) = 0;

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
};
}

#endif // ROBOTMODEL_HPP
