#ifndef TASKFRAME_HPP
#define TASKFRAME_HPP

#include <base/samples/RigidBodyState.hpp>

namespace wbc{

/**
 * @brief The TaskFrame class describes a Coordinate frame in the robot model, which is relevant for the control task,
 *        e.g. the tcp, the robots base frame or a camera frame. Each Task Frame is associated
 *        with a kinematic chain that spans from the task frame itself to the (global) base of the robot model. Each chain can contain
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

    /** Joint names in the kinematic chain associated with the task frame */
    std::vector<std::string> joint_names;
};

}

#endif // TASKFRAME_HPP
