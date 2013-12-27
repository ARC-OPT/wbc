#include "Wbc.hpp"
#include <base/logging.h>
#include "SubTask.hpp"

bool Wbc::AddSubTask(const KDL::Chain &chain,
                     const uint no_task_variables,
                     const uint priority){

    configured_ = false;
    SubTask* sub_task = new SubTask(chain, no_task_variables, priority);

    //Insert sub task into sub task map at the correct position (priority ordered)
    uint i = 0;
    for(i = 0; i < sub_task_map_.size(); i++){
        if(sub_task_map_[i]->priority_ > priority)
            break;
    }
    sub_task_map_.insert(sub_task_map_.begin() + i, sub_task);

    return true;
}

bool Wbc::AddSolver(const std::string &name,
                    HierarchicalSolver* solver){
    configured_ = false;
    return true;
}

bool Wbc::Configure(){

    joint_index_map_.clear();
    configured_ = true;
    return true;
}

void Wbc::Solve(const std::string &solver_name,
                const base::samples::Joints &status,
                base::commands::Joints &solver_output){
    if(!configured_)
        throw std::invalid_argument("Configure has not been called yet");

    //If not yet done, create joint index map
    if(joint_index_map_.empty()){
        for(uint i = 0; i < status.size(); i++)
            joint_index_map_[status.names[i]] = i;
    }

    //Update sub tasks

    //Solve

}
