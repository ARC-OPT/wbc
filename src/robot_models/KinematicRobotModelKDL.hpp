#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include <kdl/tree.hpp>
#include "RobotModel.hpp"

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

public:
    KinematicRobotModelKDL(const std::string& _base_frame = "");
    virtual ~KinematicRobotModelKDL(){}

    /**
     * @brief Add a partial model to the overall robot model, e.g. another robot or an object attached to the robot. Here, this will attach
     *        the KDL tree extracted from the urdf model to the given hook. If no model exists yet, this will be set as overall KDL model
     * @param model_file Filename of the urdf model
     * @param initial_pose Initial position and orientation of the model with respect to the hook frame
     * @param hook_name Frame to which the model shall be attached. Has to be a valid segment of the KDL tree
     */
    virtual bool loadModel(const RobotModelConfig& config);

    /**
     * @brief Update all task frames joints in the model with a new joint state.
     * @param joint_state The joint_state vector
     */
    virtual void update(const base::samples::Joints& joint_state);

    /**
     * @brief Update the pose of a link (segment) in the model. This feature can be used e.g. if there are multiple robots (to update the relative
     *        pose between them) or if there is an object in the scene (e.g. to update its pose wrt to the camera frame)
     * @param new_pose The new link pose. The SourceFrame has to be the same as the segments name that is to be updated.
     */
    virtual void update(const base::samples::RigidBodyState &new_pose);

    /**
     * @brief Check if a frame is available in the model
     */
    bool hasFrame(const std::string &name);

protected:
    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return True in case of success, false otherwise (e.g. if the task frame already exists)
     */
    virtual bool addTaskFrameInternal(const std::string &tf_name);
};

}

#endif // KINEMATICMODEL_HPP
