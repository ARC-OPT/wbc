#ifndef KINEMATICMODEL_HPP
#define KINEMATICMODEL_HPP

#include <kdl/tree.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "TaskFrame.hpp"

namespace wbc{

/** The kinematic model describes the kinemetic relationships required for wbc. It is based on a single KDL Tree, although it is possible to add an arbitrary
 *  number of trees (e.g. for controlling multiple robots at the same time or to described robot-object relationships). Particularly, the kinematic contains
 *  the task frames (see TaskFrame.hpp for details), which are used to describe the control problem.
 */
class KinematicModel{

protected:
    KDL::Tree full_tree;
    TaskFrameMap tf_map;

public:
    /**
     * @brief Add an additional KDL tree to the model. If no tree has been added yet, the root segment of the given tree will be the root of the overall tree.
     * @param tree KDL tree to add
     * @param hook The segment to which the new tree shall be attached. Has to be a valid segment of all previously added trees. If the given tree is the first tree, this parameter will be ignored.
     * @param initial_pose Initial pose of the newly added tree (initial transform from the hook segment to the root segment of the newly added tree). If this is the first, tree this parameter will be ignored.
     * @return True in case of success, false otherwise
     */
    bool addTree(const KDL::Tree &tree,
                 const base::samples::RigidBodyState& initial_pose,
                 const std::string hook = "");

    /**
     * @brief Add a task frame to the model (see TaskFrame.hpp for details about task frames)
     * @param tf_name Name of the task frame. Has to be a valid segment of the model
     * @return True in case of success, false otherwise
     */
    bool addTaskFrame(const std::string &tf_name);

    /**
     * @brief add multiple task frames at once
     */
    bool addTaskFrames(const std::vector<std::string> &task_frame_ids);

    /**
     * @brief Remove the given task frame
     */
    void removeTaskFrame(const std::string &tf_name);

    /**
     * @brief Update all joints in the model with a new joint state. This will trigger computation of all task frame poses and Jacobians
     * @param joint_state The joint_state vector has to contain all joint names that are being used by the kinematic chains associated to any
     *        task frame that has been added (this might be less or equal the number of joints in the complete KDL tree)
     */
    void updateJoints(const base::samples::Joints& joint_state);

    /**
     * @brief Update the pose of a link (segment) in the model. This feature can be used e.g. if there are multiple robots (to update the relative
     *        pose between them) or if there is an object in the scene (e.g. to update its pose wrt to the camera frame)
     * @param new_pose The new link pose. The SourceFrame has to be the same as the segments name that is to be updated.
     */
    void updateLink(const base::samples::RigidBodyState &new_pose);

    /**
     * @brief Return a task frame with the given name. Will throw in case the task frame id does not exist
     */
    const TaskFrame& getTaskFrame(const std::string& name);

    /**
     * Return the full KDL Tree of the model.
     *
     * Careful here: We use updateLink() to update segment poses in the KDL::Chains that are associated to the task frames.
     * Since it is currently not possible to update segments in a KDL tree after the tree has bee created, these updates will NOT be reflected by the KDL Tree.
     * Thus, if you call updateLink() to change the pose of a segment, the KDL Tree will NOT be up-to-date anymore.
     */
    const KDL::Tree &getTree(){return full_tree;}

    /**
     * @brief Return all task frames
     */
    const TaskFrameMap& getTaskFrameMap(){return tf_map;}

    /**
     * @brief Chech if a frame is available in the model
     */
    bool hasFrame(const std::string &name);

    /** Return root segment of the overall kinematic tree*/
    const std::string& getRootName(){return full_tree.getRootSegment()->first;}
};

}

#endif // KINEMATICMODEL_HPP
