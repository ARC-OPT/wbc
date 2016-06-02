#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include <kdl/tree.hpp>
#include "RobotModel.hpp"

namespace wbc{

/** The kinematic model describes the kinemetic relationships required for wbc. It is based on a single KDL Tree, although it is possible to add an arbitrary
 *  number of trees (e.g. for controlling multiple robots at the same time or to described robot-object relationships). Particularly, the kinematic contains
 *  the task frames (see TaskFrame.hpp for details), which are used to describe the control problem.
 */
class KinematicRobotModelKDL : public RobotModel{

protected:
    KDL::Tree full_tree;

public:
    KinematicRobotModelKDL(){}
    virtual ~KinematicRobotModelKDL(){}

    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid frame of the model
     * @return True in case of success, false otherwise
     */
    virtual bool addTaskFrame(const std::string &tf_name);

    /**
     * @brief Remove the given task frame
     */
    virtual void removeTaskFrame(const std::string &tf_name);

    /**
     * @brief Add a partial model to the overall robot model, e.g. another robot or an object attached to the robot. Here, this will attach
     *        the KDL tree extracted from the urdf model to the given hook. If no model exists yet, this will be set as overall KDL model
     * @param model_file Filename of the urdf model
     * @param initial_pose Initial position and orientation of the model with respect to the hook frame
     * @param hook_name Frame to which the model shall be attached. Has to be a valid segment of the KDL tree
     */
    virtual bool addModelFromFile(const std::string& model_file,
                                  base::samples::RigidBodyState& initial_pose,
                                  const std::string hook_name = "");

    /**
     * @brief Check if a frame is available in the model
     */
    bool hasFrame(const std::string &name);
};

}

#endif // KINEMATICMODEL_HPP
