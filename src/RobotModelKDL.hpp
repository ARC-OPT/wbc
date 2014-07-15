#ifndef ROBOTMODELKDL_HPP
#define ROBOTMODELKDL_HPP

#include "KinematicChainKDL.hpp"
#include <kdl/tree.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

class RobotModelKDL{

    typedef std::map<std::string, KinematicChainKDL*> KinChainMap;
    typedef std::map<std::string, TaskFrameKDL*> TFMap;

public:
    RobotModelKDL(const KDL::Tree& tree);
    ~RobotModelKDL();

    bool hasTaskFrame(const std::string& id){return kin_chain_map_.count(id) > 0;}
    bool addTaskFrame(const std::string& id);
    void update(const base::samples::Joints& joint_state);
    TaskFrameKDL* getTaskFrame(const std::string &id);
    std::string robotRoot(){return tree_.getRootSegment()->second.segment.getName();}

    KDL::Tree tree_;
    KinChainMap kin_chain_map_;
};
}

#endif // ROBOTMODELKDL_HPP
