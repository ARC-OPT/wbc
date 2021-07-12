#include "JointIntegrator.hpp"
#include <base-logging/Logging.hpp>

namespace wbc {

void JointIntegrator::integrate(const base::samples::Joints& joint_state, base::commands::Joints &cmd, double cycle_time, IntegrationMethod method){

    if(initialized){
        switch (method) {
        case RECTANGULAR:
            integrateRectangular(cmd,cycle_time);
            break;
        case TRAPEZOIDAL:
            integrateTrapezoidal(cmd,cycle_time);
            break;
        default:
            throw std::runtime_error("Invalid integration Method: " + std::to_string(method));
        }
    }
    else{
         for(uint i = 0; i < cmd.size(); i++){
             try{
             switch(cmdMode(cmd[i]))
             {
                 case base::JointState::SPEED:
                     cmd[i].position = joint_state[cmd.names[i]].position;
                     break;
                 case base::JointState::ACCELERATION:
                     cmd[i].position = joint_state[cmd.names[i]].position;
                     cmd[i].speed = joint_state[cmd.names[i]].speed;
                     break;
                 default:
                     throw std::runtime_error("Invalid control mode");
                 }
             }
             catch(base::samples::Joints::InvalidName e){
                 LOG_ERROR_S << "Joint " << cmd.names[i] << " is in command vector, but not in joint state"<<std::endl;
                 throw e;
             }
         }
         initialized = true;
    }
    prev_cmd = cmd;
}

void JointIntegrator::integrateRectangular(base::commands::Joints &cmd, double cycle_time){
    for(uint i = 0; i < cmd.size(); i++)
    {
        double h = cycle_time;
        double q_prev = prev_cmd[i].position;
        double q_dot_prev = prev_cmd[i].speed;
        double q_dot = cmd[i].speed;
        double q_dotdot = cmd[i].acceleration;

        switch(cmdMode(cmd[i]))
        {
        case base::JointState::SPEED:
            cmd[i].position = q_prev + q_dot*h;
            break;
        case base::JointState::ACCELERATION:
            cmd[i].speed = q_dot_prev + q_dotdot*h;
            cmd[i].position = q_prev + cmd[i].speed*h;
            break;
        default:
            throw std::runtime_error("Invalid control mode");
        }
    }
}

void JointIntegrator::integrateTrapezoidal(base::commands::Joints &cmd, double cycle_time){
    for(uint i = 0; i < cmd.size(); i++)
    {
        double h = cycle_time/2;
        double q_prev = prev_cmd[i].position;
        double qd_prev = prev_cmd[i].speed;
        double qd = cmd[i].speed;
        double qdd = cmd[i].acceleration;
        double qdd_prev = cmd[i].acceleration;

        switch(cmdMode(cmd[i]))
        {
        case base::JointState::SPEED:
            cmd[i].position = q_prev + (qd+qd_prev)*h;
            break;
        case base::JointState::ACCELERATION:
            cmd[i].speed = qd_prev + (qdd_prev+qdd)*h;
            cmd[i].position = q_prev + (cmd[i].speed+qd_prev)*h;
            break;
        default:
            throw std::runtime_error("Invalid control mode");
        }
    }
}

base::JointState::MODE JointIntegrator::cmdMode(const base::JointState &cmd){
    if(cmd.hasPosition())
        return base::JointState::POSITION;
    else if(cmd.hasSpeed() && !cmd.hasPosition())
        return base::JointState::SPEED;
    else if(cmd.hasAcceleration() && !cmd.hasSpeed() && !cmd.hasPosition())
        return base::JointState::ACCELERATION;
    else
        throw std::runtime_error("Invalid control mode");

}

}
