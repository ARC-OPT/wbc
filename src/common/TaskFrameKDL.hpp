#ifndef TASKFRAMEKDL_HPP
#define TASKFRAMEKDL_HPP

#include "TaskFrame.hpp"
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace base{
    namespace samples{
        class Joints;
    }
}

namespace wbc{

/**
 * @brief The TaskFrame class describes a Coordinate frame in the kinematic model, which is relevant for the control task,
 *        e.g. the tcp, the robots base frame or a camera frame. Each Task Frame is associated
 *        with a kinematic chain that spans from the task frame itself to the (global) base of the kinematic model. Each chain can contain
 *        an arbitrary number of joints and links. In WBC, task frames are used to describe the control problem. They define which
 *        robot parts are involved to regulated the control error and in which coordinate system the control actions takes place.
 */
class TaskFrameKDL : public TaskFrame{
public:
    TaskFrameKDL(const std::string &name);
    TaskFrameKDL(const KDL::Chain& chain, const std::string &name);

    /** Pose of the Task frame wrt the base of the robot */
    KDL::Frame pose_kdl;
    /** Jacobian relating the task frame to the base of the robot */
    KDL::Jacobian jacobian;
    /** Jacobian of the complete robot. Columns not related to a joint in this kinematic chain will be zero*/
    KDL::Jacobian full_robot_jacobian;
    /** Current joint positions in radians */
    KDL::JntArray joint_positions;
    /** Kinematic associated with the task frame and the base of the robot*/
    KDL::Chain chain;
    /** Last time sensor a joint state or pose information was updated*/
    base::Time last_update;

    /**
     * @brief Update the task frame with the current joint state.
     * @param joint_state Current joint state. Has to contain at least all joints within the kinematic chain associated with this task frame.
     *                    Names will be mapped to correct indices internally
     */
    void updateJoints(const base::samples::Joints &joint_state);

    /**
     * @brief Update the pose of a particular segment in the kinematic chain
     * @param segment_pose New segment pose. SourceFrame has to be the same as the segments name.
     */
    void updateSegment(const base::samples::RigidBodyState &new_pose);

    /**
     * @brief Recompute the kinematics information of the task frame. updateJoints has to be called at least once before this is called.
     * @param robot_joint_names The joint name vector of the complete robot
     */
    void recomputeKinematics(const std::vector<std::string> &robot_joint_names);

};
}

#endif // TASKFRAMEKDL_HPP
