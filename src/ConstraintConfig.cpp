#include "ConstraintConfig.hpp"

namespace wbc{

ConstraintConfig::ConstraintConfig() :
    activation(0),
    timeout(0){

}

void ConstraintConfig::validate() const{
    if(name.empty())
        throw std::invalid_argument("ConstraintConfig: Constraint name must not be empty!");
    if(activation < 0 || activation > 1)
        throw std::invalid_argument("Constraint " + name + ": Activation must between 0 and 1, but is " + std::to_string(activation));
    if(priority < 0)
        throw std::invalid_argument("Constraint " + name + ": Priority must >= 0, but is " + std::to_string(priority));
    if(type == cart){
        if(root.empty())
            throw std::invalid_argument("Constraint " + name + ": Root frame is empty");
        if(tip.empty())
            throw std::invalid_argument("Constraint " + name + ": Tip frame is empty");
        if(ref_frame.empty())
            throw std::invalid_argument("Constraint " + name + ": Ref frame is empty");
        if(weights.size() != 6)
            throw std::invalid_argument("Constraint " + name + ": Size of weight vector should be 6, but is " + std::to_string(weights.size()));
    }
    else if(type == jnt){
        if(weights.size() != joint_names.size())
            throw std::invalid_argument("Constraint " + name + ": Size of weight vector should be " + std::to_string(joint_names.size()) + ", but is " + std::to_string(weights.size()));
        for(size_t i = 0; i < joint_names.size(); i++)
            if(joint_names[i].empty())
                throw std::invalid_argument("Constraint " + name + ": Name of joint " + std::to_string(i) + " is empty!");
    }
    else
        throw std::invalid_argument("Constraint " + name + ": Invalid constraint type: " + std::to_string(type));

    for(size_t i = 0; i < weights.size(); i++)
        if(weights[i] < 0)
            throw std::invalid_argument("Constraint " + name + ": Constraint weights must be >= 0, but weight " + std::to_string(i) + " is " + std::to_string(weights[i]));
}

uint ConstraintConfig::noOfConstraintVariables() const{
    if(type == cart)
        return 6;
    else
        return joint_names.size();
}

}
