#include "Wbc.hpp"
#include <base/logging.h>
#include "SubTask.hpp"

bool Wbc::AddRobot(const KDL::Tree &tree){

    robot_ = tree;

    //Create joint index map
    joint_index_map_.clear();
    KDL::SegmentMap seg_map = robot_.getSegments();
    uint i = 0;
    for(KDL::SegmentMap::iterator it = seg_map.begin(); it != seg_map.end(); it++){
        KDL::Segment seg = it->second.segment;
        if(seg.getJoint().getType() != KDL::Joint::None){
            joint_index_map_[seg.getJoint().getName()] = i;
            LOG_INFO("Joint %s has index %i", seg.getJoint().getName().c_str(), i);
            i++;
        }
    }

    configured_ = false;
    return true;
}

bool Wbc::AddSubTask(const std::string &root_frame,
                     const std::string &tip_frame,
                     const uint no_task_variables,
                     const uint priority){
    configured_ = false;

    KDL::Chain chain;
    if(!robot_.getChain(root_frame, tip_frame, chain)){
        LOG_ERROR("Unable to get kinematic chain between %s and %s from robot tree", root_frame.c_str(), tip_frame.c_str());
        return false;
    }

    SubTask* sub_task = new SubTask(chain, no_task_variables);

    return true;
}


bool Wbc::AddSolver(const std::string &name,
                    HierarchicalSolver* solver){
    configured_ = false;
    return true;
}

bool Wbc::Configure(){

    configured_ = true;
    return true;
}

void Wbc::Solve(const std::string &solver_name,
                const base::samples::Joints &status,
                base::commands::Joints &solver_output){
    if(!configured_)
        throw std::invalid_argument("Configure has not been called yet");


}
