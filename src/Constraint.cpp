#include "Constraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/Float.hpp>

namespace wbc{

Constraint::Constraint(){

}

Constraint::Constraint(const ConstraintConfig& _config, uint n_robot_joints) :
    config(_config){

    unsigned int no_variables = config.nVariables();

    y_ref.resize(no_variables);
    y_ref_root.resize(no_variables);
    weights.resize(no_variables);
    weights_root.resize(no_variables);
    y_solution.resize(no_variables);
    y.resize(no_variables);
    y_solution_error.resize(no_variables);
    y_error.resize(no_variables);

    A.resize(no_variables, n_robot_joints);
    _config.validate();

    reset();
}

Constraint::~Constraint(){

}

void Constraint::reset(){

    unsigned int no_variables = config.nVariables();

    y_ref_root.setConstant(no_variables, base::NaN<double>());
    y_ref.setZero(no_variables);
    y_solution_error.setConstant(no_variables, base::NaN<double>());
    y_error.setConstant(no_variables, base::NaN<double>());
    A.setZero();
    activation = config.activation;
    for(uint i = 0; i < no_variables; i++){
        weights(i) = config.weights[i];
        weights_root(i) = config.weights[i];
    }
    // Reset timeout and time. Like this, constraints can get activated only after they received a reference value
    timeout = 1;
    time.microseconds = 0;
}

void Constraint::checkTimeout(){
    timeout = (int)time.isNull(); // If there has never been a reference value, set the constraint should to timeout
    if(config.timeout > 0)
        timeout = (int)(base::Time::now() - time).toSeconds() > config.timeout;
}

void Constraint::setWeights(const base::VectorXd& weights){
    if(config.nVariables() != weights.size()){
        LOG_ERROR("Constraint %s: Size of weight vector should be %i but is %i", config.name.c_str(), config.nVariables(), weights.size())
        throw std::invalid_argument("Invalid constraint weights");
    }

    for(uint i = 0; i < weights.size(); i++)
        if(weights(i) < 0){
            LOG_ERROR("Constraint %s: Weight values should be > 0, but weight %i is %f", config.name.c_str(), i, weights(i));
            throw std::invalid_argument("Invalid constraint weights");
        }

    this->weights = weights;
}

void Constraint::setActivation(const double activation){
    if(activation < 0 || activation > 1){
        LOG_ERROR("Constraint %s: Activation has to be between 0 and 1 but is ", config.name.c_str(), activation);
        throw std::invalid_argument("Invalid constraint activation");
    }
    this->activation = activation;
}

}// namespace wbc
