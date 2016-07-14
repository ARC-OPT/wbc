#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include <kdl/tree.hpp>
#include "RobotModel.hpp"
#include <base/samples/Joints.hpp>

namespace wbc{

class TaskFrameKDL;

/** The kinematic model describes the kinemetic relationships required for wbc. It is based on a single KDL Tree, although it is possible to add an arbitrary
 *  number of trees (e.g. for controlling multiple robots at the same time or to described robot-object relationships). Particularly, the kinematic contains
 *  the task frames (see TaskFrame.hpp for details), which are used to describe the control problem.
 */
class KinematicRobotModelKDL : public RobotModel{

    typedef std::map<std::string, int> JointIndexMap;

protected:
    KDL::Tree full_tree;
    JointIndexMap joint_index_map;
    base::samples::Joints current_joint_state;

public:
    KinematicRobotModelKDL();
    virtual ~KinematicRobotModelKDL(){}

    /**
     * @brief Load all robot models and add them to the overall KDL tree. Also, add all task frames.
     * @param robot_model_config Config of the robot models. For each element in this vector loadURDFModel() will be called
     * @param task_frame_ids IDs of all task frames that are required. For each element in this vector addTaskFrame will be called.
     * @return True in case of success, false otherwise
     */
    virtual bool configure(const std::vector<RobotModelConfig>& robot_model_config,
                           const std::vector<std::string>& task_frame_ids,
                           const std::string &_base_frame);


    /**
     * @brief Update all task frames in the model with a new joint state and (optionally) new link states
     * @param joint_state The joint_state vector. Has to contain all joint names that are relevant to any task frame,
     *                    that means all joints that are contained in a kinematic chain between the robot base frame and
     *                    one of the task frames
     * @param poses The new link poses. The SourceFrame has to be the same as the segment name that is to be updated. Can be left empty,
     *              if no segmented are to be updated
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>());


    /**
     * @brief getState Return the relative state of two task frames that are defined by source and target frame of the input
     * @param tf_one_name Name of the first task frame
     * @param tf_two_name Name of the second task frame
     * @param state Relative pose (twist and acceleration). E.g. the computed pose will be the second task frame wrt to the first task frame
     */
    virtual void getState(const std::string& tf_one_name,
                          const std::string& tf_two_name,
                          base::samples::RigidBodyState& state);

    /**
     * @brief getState Return the state of the joints given by joint names
     * @param joint_names Joint names to evaluated
     * @param state Position, (velocity and acceleration) of the given joints
     */
    virtual void getState(const std::vector<std::string> &joint_names,
                          base::samples::Joints& state);

    /**
     * @brief Check if a frame is available in the model
     */
    bool hasFrame(const std::string &name);

protected:
    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the KDL tree. This method will try to create a kinematic
     *                chain between the robot base frame and the task frame.
     * @return True in case of success, false otherwise (e.g. if the task frame already exists)
     */
    bool addTaskFrame(const std::string &tf_name);

    /**
     * @brief loadURDFModel Load a URDF model and add it to frame denoted by <hook>. If the KDL tree is empty, it will be replaced by the
     *                      Tree parsed from this URDF model
     * @param model Robot model config. Each config has the following members:
     *
     *            - file: Has to be the full path of a URDF model file. Will be parsed into a KDL tree. If the overall tree
     *                    is empty (i.e. if loadModel is called for the first time), the overall tree will be equal to the new
     *                    tree afterwards. Otherwise the new tree will be attached to the overall tree.
     *            - hook: The KDL segment name to which the new tree will be attached. Can be left empty if loadModel() is called
     *                    for the first time
     *            - initial_pose: Initial pose of the new KDL tree with respect to the given hook frame. Will be ignored if loadModel()
     *                            is called for the first time
     *
     * @return True in case of success, false otherwise (e.g. if the urdf model cannot be parsed)
     */
    bool loadURDFModel(const RobotModelConfig& model);

    /**
     * Cleanup, so that the robot model can be configured again
     */
    void clear();
};

}

#endif // KINEMATICMODEL_HPP
