#ifndef CONSTRAINT_CPP
#define CONSTRAINT_CPP

#include "Constraint.hpp"

namespace wbc{

Constraint::Constraint(){
}

Constraint::Constraint(const ConstraintConfig& _config){

    config = _config;

    if(config.type == jnt)
        no_variables = _config.joint_names.size();
    else
        no_variables = 6;

    if(config.weights.size() != no_variables){
        LOG_ERROR("Constraint '%s' has %i variables, but its weights vector has size %i", config.name.c_str(), no_variables, config.weights.size());
        throw std::invalid_argument("Invalid WBC config");
    }
    if(config.activation < 0 || config.activation > 1){
        LOG_ERROR("Activation of constraint '%s' is %f. It has to be be between 0 and 1", config.name.c_str(),config.activation);
        throw std::invalid_argument("Invalid WBC config");
    }

    for(uint i = 0; i < config.weights.size(); i++){
        if(config.weights[i] < 0){
            LOG_ERROR("Weight no %i of constraint '%s' is %f. It has to be >= 0",
                      i, config.name.c_str(), config.weights[i]);
            throw std::invalid_argument("Invalid WBC config");

        }
    }

    y_ref.resize(no_variables);
    y_ref_root.resize(no_variables);
    weights.resize(no_variables);
    weights_root.resize(no_variables);
    y_solution.resize(no_variables);
    y.resize(no_variables);

    reset();
}

void Constraint::reset()
{
    y_ref_root.setConstant(base::NaN<double>());
    y_ref.setZero();
    activation = config.activation;
    for(uint i = 0; i < no_variables; i++){
        weights(i) = config.weights[i];
        weights_root(i) = config.weights[i];
    }

     //Set timeout to true in the beginning. Like this, Constraints have to get a
    //reference value first to be activated, independent of the activation value
    constraint_timed_out = 1;
}

} //namespace wbc
#endif // Constraint_CPP
