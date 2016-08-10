#include "ConstraintConfig.hpp"
#include <base/Logging.hpp>

namespace wbc{

ConstraintConfig::ConstraintConfig(){
    timeout = 0;
    activation = 0;
}

bool ConstraintConfig::isValid() const{

    if(name.empty()){
        LOG_ERROR("ConstraintConfig: Constraint name must not be empty!");
        return false;
    }
    if(activation < 0 || activation > 1){
        LOG_ERROR("Constraint %s: Activation must between 0 and 1, but is %f", name.c_str(), activation);
        return false;
    }
    if(priority < 0){
        LOG_ERROR("Constraint %s: Priority must >= 0, but is %i", name.c_str(), priority);
        return false;
    }
    if(type == cart){
        if(root.empty()){
            LOG_ERROR("Constraint %s: Root frame is empty", name.c_str());
            return false;
        }
        if(tip.empty()){
            LOG_ERROR("Constraint %s: Tip frame is empty", name.c_str());
            return false;
        }
        if(ref_frame.empty()){
            LOG_ERROR("Constraint %s: Ref frame is empty", name.c_str());
            return false;
        }
        if(weights.size() != 6){
            LOG_ERROR("Constraint %s: Weight size is expected to be %i, but is %i", name.c_str(), 6, weights.size());
            return false;
        }
    }
    else if(type == jnt){
        if(weights.size() != joint_names.size()){
            LOG_ERROR("Constraint %s: Weight size is expected to be %i, but is %i", name.c_str(), joint_names.size(), weights.size());
            return false;
        }
        for(size_t i = 0; i < joint_names.size(); i++){
            if(joint_names[i].empty()){
                LOG_ERROR("Constraint %s: Joint name %i is empty", name.c_str(), i);
                return false;
            }
        }
    }
    else{
        LOG_ERROR("Constraint %s: Invalid constraint type", name.c_str());
        return false;
    }

    for(size_t i = 0; i < weights.size(); i++){
        if(weights[i] < 0){
            LOG_ERROR("Constraint %s: Weight values must be >= 0, but weight %i is %f", name.c_str(), i, weights[i]);
            return false;
        }
    }

    return true;
}

}
