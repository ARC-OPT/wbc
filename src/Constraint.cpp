#ifndef CONSTRAINT_CPP
#define CONSTRAINT_CPP

#include "Constraint.hpp"

namespace wbc{

void Constraint::setReference(const base::samples::RigidBodyState &ref){
    if(config.type != cart){
        LOG_ERROR("Reference input of constraint %s is Cartesian but constraint is not Cartesian", config.name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(!ref.hasValidVelocity() ||
            !ref.hasValidAngularVelocity()){
        LOG_ERROR("Reference input of constraint %s has invalid velocity and/or angular velocity", config.name.c_str());
        throw std::invalid_argument("Invalid Cartesian reference input");
    }

    y_ref.segment(0,3) = ref.velocity;
    y_ref.segment(3,3) = ref.angular_velocity;

    updateTime();
}

void Constraint::setReference(const base::samples::Joints& ref){
    if(config.type != jnt){
        LOG_ERROR("Reference input of constraint %s is in joint space but constraint is not in joint space", config.name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(ref.size() != config.joint_names.size()){
        LOG_ERROR("Size for input reference of constraint %s should be %i but is %i", config.name.c_str(), config.joint_names.size(), ref.size());
        throw std::invalid_argument("Invalid joint reference input");
    }

    for(uint i = 0; i < config.joint_names.size(); i++){
        uint idx;
        try{
            idx = ref.mapNameToIndex(config.joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Constraint::setReference: Constraint with name %s expects joint %s, but this joint is not in reference vector!", config.name.c_str(), config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid joint reference input");
        }

        if(!ref[idx].hasSpeed()){
            LOG_ERROR("Reference input for joint %s of constraint %s has invalid speed value(s)", ref.names[idx].c_str(), config.name.c_str());
            throw std::invalid_argument("Invalid joint reference input");
        }

        y_ref(i) = ref[idx].speed;
    }

    updateTime();
}

void Constraint::updateTime(){
    last_ref_input = base::Time::now();;
}

void Constraint::validate()
{
    if(activation < 0 || activation > 1){
        LOG_ERROR("Sub constraint: %s. Activation must be >= 0 and <= 1, but is %f", config.name.c_str(), activation);
        throw std::invalid_argument("Invalid activation value");
    }
    if(config.type == jnt){
        if(weights.size() !=  (int)config.joint_names.size()){
            LOG_ERROR("Size of weight vector is %f, but constraint %s has %f constraint variables", weights.size(), config.name.c_str(), config.joint_names.size());
            throw std::invalid_argument("Invalid no of constraint weights");
        }
    }
    else{
        if(weights.size() !=  6){
            LOG_ERROR("Size of weight vector is %f, but constraint %s has %f constraint variables", weights.size(), 6, config.joint_names.size());
            throw std::invalid_argument("Invalid no of constraint weights");
        }
    }
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
    A.setZero();
    last_ref_input = base::Time::now();
    resetEvaluation();
}

void Constraint::resetEvaluation(){

    y_solution.setConstant(base::NaN<double>());
    y.setConstant(base::NaN<double>());
    error_y_solution.setZero();
    error_y.setZero();
    sqrt_sum_error_y_solution.setZero();
    sqrt_sum_error_y.setZero();
    n_samples.setZero();
    mask.setOnes();
}

void Constraint::evaluate(const base::VectorXd &solver_output, const base::VectorXd &actual_robot_vel){
    time = base::Time::now();

    // Set Errors to zero if the constraint is deactivated
    if(activation == 0){
        resetEvaluation();
        return;
    }

    mask.setOnes();

    // Also set errors of single constraint variables to zero if the corresponding weight is zero
    for(uint i = 0; i < no_variables; i++){
        if( weights(i) == 0)
            mask(i) = 0;
    }

    n_samples = n_samples + mask;

    y_solution       = A * solver_output;
    y                = A * actual_robot_vel;
    error_y_solution = mask.cwiseProduct(y_ref_root - y_solution);
    error_y          = mask.cwiseProduct(y_ref_root - y);
    VectAbs(error_y_solution);
    VectAbs(error_y);

    sqrt_sum_error_y_solution += error_y_solution.cwiseProduct(error_y_solution);
    sqrt_sum_error_y          += error_y.cwiseProduct(error_y);

    for(uint i = 0; i < no_variables; i++){
        if(n_samples(i) == 0)
            rms_error_y_solution(i) = 0;
        else
            rms_error_y_solution(i) = sqrt_sum_error_y_solution(i) / n_samples(i);
        rms_error_y_solution(i) = sqrt(rms_error_y_solution(i));
    }

    for(uint i = 0; i < no_variables; i++){
        if(n_samples(i) == 0)
            rms_error_y(i) = 0;
        else
            rms_error_y(i) = sqrt_sum_error_y(i) / n_samples(i);
        rms_error_y(i) = sqrt(rms_error_y(i));
    }
}
void Constraint::VectAbs(base::VectorXd& in){
    for(uint i = 0; i < in.size(); i++)
        in(i) = fabs(in(i));
}

} //namespace wbc
#endif // Constraint_CPP
