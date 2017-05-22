#include "ConstraintConfig.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

ConstraintConfig::ConstraintConfig() :
    activation(0),
    timeout(0){

}

void ConstraintConfig::validate() const{
    if(name.empty()){
        LOG_ERROR("ConstraintConfig: Constraint name must not be empty!");
        throw std::invalid_argument("Invalid constraint config");}
    if(activation < 0 || activation > 1){
        LOG_ERROR("Constraint %s: Activation has to be between 0 and 1 but is ", name.c_str(), activation);
        throw std::invalid_argument("Invalid constraint config");}
    if(priority < 0){
        LOG_ERROR("Constraint %s: Priority has to be >= 0, but is %i", name.c_str(), priority);
        throw std::invalid_argument("Invalid constraint config");}
    if(type == cart){
        if(root.empty()){
            LOG_ERROR("Constraint %s: Root frame is empty", name.c_str());
            throw std::invalid_argument("Invalid constraint config");}
        if(tip.empty()){
            LOG_ERROR("Constraint %s: Tip frame is empty", name.c_str());
            throw std::invalid_argument("Invalid constraint config");}
        if(ref_frame.empty()){
            LOG_ERROR("Constraint %s: Ref frame is empty", name.c_str());
            throw std::invalid_argument("Invalid constraint config");}
        if(weights.size() != 6){
            LOG_ERROR("Constraint %s: Size of weight vector should be 6, but is %i", name.c_str(), weights.size());
            throw std::invalid_argument("Invalid constraint config");}
    }
    else if(type == jnt){
        if(weights.size() != joint_names.size()){
            LOG_ERROR("Constraint %s: Size of weight vector should be %i, but is %i", name.c_str(), joint_names.size(), weights.size());
            throw std::invalid_argument("Invalid constraint config");}
        for(size_t i = 0; i < joint_names.size(); i++)
            if(joint_names[i].empty()){
                LOG_ERROR("Constraint %s: Name of joint %i is empty", name.c_str(), i);
                throw std::invalid_argument("Invalid constraint config");}
    }
    else{
        LOG_ERROR("Constraint %s: Invalid constraint type. Allowed types are 'jnt' and 'cart'", name.c_str());
        throw std::invalid_argument("Invalid constraint config");}

    for(size_t i = 0; i < weights.size(); i++)
        if(weights[i] < 0){
            LOG_ERROR("Constraint %s: Constraint weights must be >= 0, but weight %i is %f", name.c_str(), i, weights[i]);
            throw std::invalid_argument("Invalid constraint config");}
}

uint ConstraintConfig::noOfConstraintVariables() const{
    if(type == cart)
        return 6;
    else
        return joint_names.size();
}

}
