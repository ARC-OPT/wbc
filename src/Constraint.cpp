#include "Constraint.hpp"

namespace wbc{

Constraint::Constraint(){

}

Constraint::Constraint(const ConstraintConfig& _config, uint n_robot_joints) :
    config(_config),
    no_variables(_config.noOfConstraintVariables()){

    y_ref.resize(no_variables);
    y_ref_root.resize(no_variables);
    weights.resize(no_variables);
    weights_root.resize(no_variables);
    y_solution.resize(no_variables);
    y.resize(no_variables);

    A.resize(no_variables, n_robot_joints);
    _config.validate();

    reset();
}

Constraint::~Constraint(){

}

void Constraint::reset(){
    y_ref_root.setConstant(no_variables, base::NaN<double>());
    y_ref.setZero(no_variables);
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
    if(no_variables != weights.size())
        throw std::invalid_argument("Constraint " + config.name + ": Size of weight vector should be "
                                    + std::to_string(no_variables) + " but is " + std::to_string(weights.size()));

    for(uint i = 0; i < weights.size(); i++)
        if(weights(i) < 0)
            throw std::invalid_argument("Constraint " + config.name + ": Weights have to be >= 0, but weight "
                                        + std::to_string(i) + " is " + std::to_string(weights(i)));

    this->weights = weights;
}

void Constraint::setActivation(const double activation){

    if(activation < 0 || activation > 1)
        throw std::invalid_argument("Constraint " + config.name + ": Activation has to be between 0 and 1 but is "
                                    + std::to_string(activation));
    this->activation = activation;
}

}// namespace wbc
