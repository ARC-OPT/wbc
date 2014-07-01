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

    y_des.segment(0,3) = ref.velocity;
    y_des.segment(3,3) = ref.angular_velocity;

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
        if(!ref[i].hasSpeed()){
            LOG_ERROR("Reference input for joint %s of constraint %s has invalid speed value(s)", ref.names[i].c_str(), config.name.c_str());
            throw std::invalid_argument("Invalid joint reference input");
        }
        y_des(i) = ref[i].speed;
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

void Constraint::computeDebug(const base::VectorXd& solver_output, const base::VectorXd& robot_vel){
    y_solution = A * solver_output;
    y = A * robot_vel;
}

void Constraint::reset()
{
    y_des.setZero();
    y_des_root_frame.setZero();
    y_solution.setZero();
    y.setZero();
    weights.setOnes();
    if(constraints_initially_active)
        activation = 1;
    else
        activation = 0;
    constraint_timed_out = 0;

    A.setZero();
    last_ref_input = base::Time::now();
    manipulability = base::NaN<double>();
}
} //namespace wbc
#endif // Constraint_CPP
