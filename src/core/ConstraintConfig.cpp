#include "TaskConfig.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

ConstraintConfig::ConstraintConfig() :
    activation(0),
    timeout(0),
    priority(-1),
    type(unset){

}

ConstraintConfig::ConstraintConfig(const std::string &name,
                                   const int priority,
                                   const std::string root,
                                   const std::string tip,
                                   const std::string ref_frame,
                                   const double activation,
                                   const std::vector<double> weights,
                                   const double timeout) :
    name(name),
    type(cart),
    priority(priority),
    weights(weights),
    activation(activation),
    timeout(timeout),
    root(root),
    tip(tip),
    ref_frame(ref_frame){
}

ConstraintConfig::ConstraintConfig(const std::string &name,
                                   const int priority,
                                   const std::vector<std::string> joint_names,
                                   const std::vector<double> weights,
                                   const double activation,
                                   const double timeout) :
    name(name),
    type(jnt),
    priority(priority),
    joint_names(joint_names),
    weights(weights),
    activation(activation),
    timeout(timeout){
}

ConstraintConfig::ConstraintConfig(const std::string &name,
                                   const int priority,
                                   const std::vector<double> weights,
                                   const double activation,
                                   const double timeout) :
    name(name),
    type(com),
    priority(priority),
    weights(weights),
    activation(activation),
    timeout(timeout){
}

ConstraintConfig::~ConstraintConfig(){
}

void ConstraintConfig::validate() const{
    if(name.empty()){
        LOG_ERROR("TaskConfig: task name must not be empty!");
        throw std::invalid_argument("Invalid task config");}
    if(activation < 0 || activation > 1){
        LOG_ERROR("Task %s: Activation has to be between 0 and 1 but is ", name.c_str(), activation);
        throw std::invalid_argument("Invalid task config");}
    if(priority < 0){
        LOG_ERROR("Task %s: Priority has to be >= 0, but is %i", name.c_str(), priority);
        throw std::invalid_argument("Invalid task config");}
    if(type == cart){
        if(root.empty()){
            LOG_ERROR("Task %s: Root frame is empty", name.c_str());
            throw std::invalid_argument("Invalid task config");}
        if(tip.empty()){
            LOG_ERROR("Task %s: Tip frame is empty", name.c_str());
            throw std::invalid_argument("Invalid task config");}
        if(ref_frame.empty()){
            LOG_ERROR("Task %s: Ref frame is empty", name.c_str());
            throw std::invalid_argument("Invalid task config");}
        if(weights.size() != 6){
            LOG_ERROR("Task %s: Size of weight vector should be 6, but is %i", name.c_str(), weights.size());
            throw std::invalid_argument("Invalid task config");}
    }
    else if(type == com){
        if(weights.size() != 3){
            LOG_ERROR("Constraint %s: Size of weight vector should be 3, but is %i", name.c_str(), weights.size());
            throw std::invalid_argument("Invalid constraint config");}
    }
    else if(type == jnt){
        if(weights.size() != joint_names.size()){
            LOG_ERROR("Task %s: Size of weight vector should be %i, but is %i", name.c_str(), joint_names.size(), weights.size());
            throw std::invalid_argument("Invalid task config");}
        for(size_t i = 0; i < joint_names.size(); i++)
            if(joint_names[i].empty()){
                LOG_ERROR("Task %s: Name of joint %i is empty", name.c_str(), i);
                throw std::invalid_argument("Invalid task config");}
    }
    else{
        LOG_ERROR("Task %s: Invalid task type. Allowed types are 'jnt' and 'cart'", name.c_str());
        throw std::invalid_argument("Invalid task config");}

    for(size_t i = 0; i < weights.size(); i++)
        if(weights[i] < 0){
            LOG_ERROR("Task %s: task weights must be >= 0, but weight %i is %f", name.c_str(), i, weights[i]);
            throw std::invalid_argument("Invalid task config");}
}

unsigned int ConstraintConfig::nVariables() const{
    if(type == cart)
        return 6;
    else if(type == com)
        return 3;
    else
        return joint_names.size();
}

}
