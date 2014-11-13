#ifndef ROBOTMODELKDL_HPP
#define ROBOTMODELKDL_HPP

#include "KinematicChainKDL.hpp"
#include <kdl/tree.hpp>

namespace wbc{

typedef std::map<std::string, KinematicChainKDL*> KinChainMap;
typedef std::map<std::string, TaskFrameKDL*> TFMap;

class RobotModelKDL{

public:
    RobotModelKDL(const KDL::Tree& tree);
    virtual ~RobotModelKDL();

    bool hasTaskFrame(const std::string& id){return kin_chain_map_.count(id) > 0;}
    bool addTaskFrames(const std::vector<std::string>& task_frame_ids);
    bool addTaskFrame(const std::string& id);
    void update(const base::samples::Joints& joint_state);
    TaskFrameKDL* getTaskFrame(const std::string &id);
    void getTFVector(std::vector<TaskFrameKDL>& task_frames);
    std::string robotRoot(){return tree_.getRootSegment()->second.segment.getName();}

    virtual void addKinChain(const KDL::Chain& chain, const std::string &id);

    KDL::Tree tree_;
    KinChainMap kin_chain_map_;
    std::vector<TaskFrameKDL*> tf_vector_;
};

class RobotModelKDLDyn : public RobotModelKDL{
public:
    RobotModelKDLDyn(const KDL::Tree& tree, const Eigen::Vector3d& gravity) :
        RobotModelKDL(tree),
        gravity_(gravity){}
    ~RobotModelKDLDyn(){}

    virtual void addKinChain(const KDL::Chain& chain, const std::string &id)
    {
        KinematicChainKDLDyn* tf_chain_kdl = new KinematicChainKDLDyn(chain, gravity_);
        kin_chain_map_[id] = tf_chain_kdl;
        tf_vector_.push_back(tf_chain_kdl->tf);
    }

    Eigen::Vector3d gravity_;

};
}

#endif // ROBOTMODELKDL_HPP
