#ifndef ROBOTMODELKDL_HPP
#define ROBOTMODELKDL_HPP

#include "TaskFrameKDL.hpp"
#include <kdl/tree.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

typedef std::map<std::string, TaskFrameKDL*> TaskFrameKDLMap;

class RobotModelKDL{
public:
    RobotModelKDL(const KDL::Tree& tree, const std::vector<std::string>& joint_names);
    ~RobotModelKDL();

    bool addTaskFrame(const std::string& id);
    void update(const base::samples::Joints& joint_state);
    uint getNoOfJoints(){return joint_index_map_.size();}
    TaskFrameKDL* getTaskFrame(const std::string &id);

    KDL::Tree tree_;
    TaskFrameKDLMap tf_kdl_map_;
    JointIndexMap joint_index_map_;
};
}

#endif // ROBOTMODELKDL_HPP
