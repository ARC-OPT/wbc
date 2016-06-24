#include "Wbc.hpp"
#include "common/Constraint.hpp"
#include "common/TaskFrame.hpp"

namespace wbc{

Wbc::Wbc(){

}

Wbc::~Wbc(){

}

Constraint* Wbc::getConstraint(const std::string& name){

    for(size_t i = 0; i < constraints.size(); i++){
        for(size_t j = 0; j < constraints[i].size(); j++){
            if(constraints[i][j]->config.name == name)
                return constraints[i][j];
        }
    }
    LOG_ERROR("No such constraint: %s", name.c_str());
    throw std::invalid_argument("Invalid constraint name");
}

std::vector<int> Wbc::getConstraintVariablesPerPrio(){
    std::vector<int> nc_pp(constraints.size());
    for(size_t i = 0; i < constraints.size(); i++){
        nc_pp[i] = 0;
        for(size_t j = 0; j < constraints[i].size(); j++)
            nc_pp[i] += constraints[i][j]->no_variables;
    }
    return nc_pp;
}

std::vector<std::string> Wbc::getTaskFrameIDs(const std::vector<ConstraintConfig>& wbc_config){

    std::vector<std::string> ids;
    for(size_t i = 0; i < wbc_config.size(); i++)
    {
        if(wbc_config[i].type == cart)
        {
            if (std::find(ids.begin(), ids.end(), wbc_config[i].root) == ids.end())
                ids.push_back(wbc_config[i].root);

            if (std::find(ids.begin(), ids.end(), wbc_config[i].tip) == ids.end())
                ids.push_back(wbc_config[i].tip);

            if (std::find(ids.begin(), ids.end(), wbc_config[i].ref_frame) == ids.end())
                ids.push_back(wbc_config[i].ref_frame);
        }
    }

    return ids;
}

TaskFrame* Wbc::getTaskFrameByName(const std::vector<TaskFrame*> task_frames, const std::string& tf_name){
    for(size_t i = 0; i < task_frames.size(); i++){
        if(task_frames[i]->name == tf_name)
            return task_frames[i];
    }
    LOG_ERROR("No such task frame in tf vector: %s", tf_name.c_str());
    throw std::invalid_argument("Invalid tf name");
}
}
