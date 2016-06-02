#ifndef TASKFRAME_HPP
#define TASKFRAME_HPP

#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

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
    TaskFrame(const std::string &name) : name(name){
    }

    /** Name of the task frame*/
    std::string name;

    /** Pose of the Task frame wrt the base of the robot */
    base::samples::RigidBodyState pose;

    /**
     * @brief Update the task frame with the current joint state
     * @param joint_state Current joint state. Has to contain at least all joints within the kinematic chain associated with this task frame.
     * Names will be mapped to correct indices internally
     */
    virtual void update(const base::samples::Joints &joint_state) = 0;

    /**
     * @brief Update the pose of a particular segment in the kinematic chain
     * @param segment_pose New segment pose. SourceFrame has to be the same as the segments name.
     */
    virtual void update(const base::samples::RigidBodyState &new_pose) = 0;
};

}

#endif // TASKFRAME_HPP
