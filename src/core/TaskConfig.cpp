#include "TaskConfig.hpp"
#include "../tools/Logger.hpp"

namespace wbc{

TaskConfig::TaskConfig(const std::string &name,
                       const int priority,
                       const std::vector<double> weights,
                       const double activation) :
    name(name),
    priority(priority),
    weights(weights),
    activation(activation){
}

TaskConfig::~TaskConfig(){
}

bool TaskConfig::isValid() const{
    if(name.empty()){
        log(logERROR)<<"TaskConfig: task name must not be empty!";
        return false;}
    if(activation < 0 || activation > 1){
        log(logERROR)<<"Task "<<name<<" Activation has to be between 0 and 1 but is " << activation;
        return false;}
    if(priority < 0){
        log(logERROR)<<"Task "<<name<<": Priority has to be >= 0, but is "<<priority;
        return false;}
    if(weights.empty()){
        log(logERROR)<<"Task "<<name<<": Weights are empty";
        return false;}
    for(size_t i = 0; i < weights.size(); i++)
        if(weights[i] < 0){
            log(logERROR)<<"Task "<<name<<": task weights must be >= 0, but weight "<<i<<" is "<<weights[i];
            return false;}

    return true;
}

}
