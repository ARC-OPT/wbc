#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <kdl/tree.hpp>
#include <base/samples/joints.h>

namespace wbc{

class TaskFrame;

typedef std::map<std::string, int> JointIndexMap;
typedef std::map<std::string, TaskFrame*> TaskFrameMap;

class RobotModel{
public:
    RobotModel(const KDL::Tree tree);
    ~RobotModel();

    bool addTaskFrame(const std::string &frame_id);
    TaskFrame* getTaskFrame(const std::string& frame_id);
    void update(const base::samples::Joints &status);

    TaskFrameMap task_frame_map_;
    JointIndexMap joint_index_map_;
    KDL::Tree tree_;
    uint no_of_joints_;
};
}

#endif
