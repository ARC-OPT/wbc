#include "Wbc.hpp"
#include "Constraint.hpp"
#include "TaskFrame.hpp"
#include <map>

namespace wbc{

Wbc::Wbc(){
}

Wbc::~Wbc(){
}

void Wbc::setConstraintWeights(const std::string &name,
                                  const base::VectorXd& weights){

    Constraint* constraint = getConstraint(name);

    if(constraint->no_variables != weights.size()){
        LOG_ERROR("Size of weight vector is %f, but constraint %s has %f constraint variables",
                  weights.size(), name.c_str(), constraint->no_variables);
        throw std::invalid_argument("Invalid constraint weights");
    }

    for(uint i = 0; i < constraint->no_variables; i++){
        if(weights(i) < 0){
            LOG_ERROR("All weight values of a constraint have to be >= 0, but weight %i of constraint %s is %f",
                      i, name.c_str(), weights(i));
            throw std::invalid_argument("Invalid constraint weights");
        }
    }

    constraint->weights = weights;
}

void Wbc::setConstraintActivation(const std::string &name,
                                     const double activation){

    Constraint* constraint = getConstraint(name);

    if(activation < 0 || activation > 1){
        LOG_ERROR("Activation values must be between 0 and 1, but activation of constraint %s is %f",
                  name.c_str(), activation);
        throw std::invalid_argument("Invalid activation value");
    }

    constraint->activation = activation;
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

std::vector<ConstraintsPerPrio> Wbc::getConstraints(){
    constraint_vector.resize(constraints.size());
    for(size_t i = 0; i < constraints.size(); i++){
        constraint_vector[i].resize(constraints[i].size());
        for(size_t j = 0; j < constraints[i].size(); j++)
            constraint_vector[i][j] = *constraints[i][j];
    }
    return constraint_vector;
}

bool Wbc::hasConstraint(const std::string &name){
    for(size_t i = 0; i < constraints.size(); i++){
        for(size_t j = 0; j < constraints[i].size(); j++){
            if(constraints[i][j]->config.name == name)
                return true;
        }
    }
    return false;
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

void Wbc::sortConfigByPriority(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config){

    // Get highest prio
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++){

        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    sorted_config.resize(max_prio + 1);

    for(uint i = 0; i < config.size(); i++)
        sorted_config[config[i].priority].push_back(config[i]);

    // Erase empty priorities
    for(uint i = 0; i < sorted_config.size(); i++){
        if(sorted_config[i].empty())
        {
            sorted_config.erase(sorted_config.begin() + i, sorted_config.begin() + i + 1);
            i--;
        }
    }
}

bool Wbc::isValid(const std::vector<ConstraintConfig> &config){

    // Check if constraint names are unique
    std::map<std::string, int> constraint_name_map;
    for(size_t i = 0; i < config.size(); i++){
        if(constraint_name_map.count(config[i].name) > 0){
            LOG_ERROR("Constraint with name %s exists more than once in wbc config", config[i].name.c_str());
            return false;
        }
        constraint_name_map[config[i].name] = i;
    }

    for(size_t i = 0; i < config.size(); i++){
        if(!config[i].isValid())
            return false;
    }

    return true;
}

}
