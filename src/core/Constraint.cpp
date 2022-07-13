#include "Task.hpp"
#include <base-logging/Logging.hpp>
#include <base/Float.hpp>

namespace wbc{

Constraint::Constraint(){

}

Constraint::Constraint(const TaskConfig& _config, uint n_robot_joints) :
    config(_config){

    unsigned int no_variables = config.nVariables();

    y_ref.resize(no_variables);
    y_ref_root.resize(no_variables);
    weights.resize(no_variables);
    weights_root.resize(no_variables);

    A.resize(no_variables, n_robot_joints);
    Aw.resize(no_variables, n_robot_joints);
    reset();
}

Constraint::~Constraint(){

}

void Constraint::reset(){

    unsigned int no_variables = config.nVariables();

    y_ref_root.setConstant(no_variables, base::NaN<double>());
    y_ref.setZero(no_variables);
    A.setZero();
    Aw.setZero();
    activation = config.activation;
    for(uint i = 0; i < no_variables; i++){
        weights(i) = config.weights[i];
        weights_root(i) = config.weights[i];
    }
    // Reset timeout and time. Like this, tasks can get activated only after they received a reference value
    timeout = 1;
    time.microseconds = 0;
}

void Constraint::checkTimeout(){
    timeout = (int)time.isNull(); // If there has never been a reference value, set the task to timeout
    if(config.timeout > 0)
        timeout = (int)(base::Time::now() - time).toSeconds() > config.timeout;
}

void Constraint::setWeights(const base::VectorXd& weights){
    if(config.nVariables() != weights.size()){
        LOG_ERROR("Task %s: Size of weight vector should be %i but is %i", config.name.c_str(), config.nVariables(), weights.size())
        throw std::invalid_argument("Invalid task weights");
    }

    for(uint i = 0; i < weights.size(); i++)
        if(weights(i) < 0){
            LOG_ERROR("Task %s: Weight values should be > 0, but weight %i is %f", config.name.c_str(), i, weights(i));
            throw std::invalid_argument("Invalid task weights");
        }

    this->weights = weights;
}

void Constraint::setActivation(const double activation){
    if(activation < 0 || activation > 1){
        LOG_ERROR("Task %s: Activation has to be between 0 and 1 but is %f", config.name.c_str(), activation);
        throw std::invalid_argument("Invalid task activation");
    }
    this->activation = activation;
}

}// namespace wbc
