#ifndef TASKFRAME_HPP
#define TASKFRAME_HPP

#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <map>

namespace wbc{

/**
 * @brief The TaskFrame class describes a Coordinate frame in the kinematic model, which is relevant for the control task,
 *        e.g. the tcp, the robots base frame or a camera frame. Each Task Frame is associated
 *        with a kinematic chain that spans from the task frame itself to the (global) base of the kinematic model. Each chain can contain
 *        an arbitrary number of joints and links. In WBC, task frames are used to describe the control problem. They define which
 *        robot parts are involved to regulated the control error and in which coordinate system the control actions takes place.
 */
class TaskFrame{
public:
    TaskFrame(){}
    TaskFrame(const KDL::Chain& chain);

    /** Pose of the Task frame wrt the base of the robot */
    KDL::Frame pose;
    /** Jacobian associated with task frame wrt to the base of the robot */
    KDL::Jacobian jacobian;
    /** Joint names in the kinematic chain associated with the task frame */
    std::vector<std::string> joint_names;
    /** Current joint positions in radians */
    KDL::JntArray joint_positions;
    /** Kinematic associated with the task frame and the base of the robot*/
    KDL::Chain chain;

    /**
     * @brief Update the task frame with the current joint state. This will trigger computation of the Task Frame's pose and the Jacobian
     * @param joint_state Current joint state. Has to contain at least all joints within the kinematic chain associated with this task frame.
     * Names will be mapped to correct indices internally
     */
    void updateJoints(const base::samples::Joints &joint_state);

    /**
     * @brief Update the pose of a particular segment in the kinematic chain
     * @param segment_pose New segment pose. SourceFrame has to be the same as the segments name.
     */
    void updateLink(const base::samples::RigidBodyState &new_pose);

    /**
     * @brief Returns root frame of the chain associated with this task frame
     */
    const std::string& rootFrame() const;

    /**
     * @brief Returns tip frame of the chain associated with this task frame
     */
    const std::string& tipFrame() const;
};

typedef std::map<std::string, TaskFrame> TaskFrameMap;

}

#endif // TASKFRAME_HPP
