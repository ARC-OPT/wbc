#include "JointIntegrator.hpp"

namespace wbc {

void JointIntegrator::integrate(const base::samples::Joints& joint_state, base::commands::Joints &cmd, double cycle_time, IntegrationMethod method){

    if(initialized){
        switch (method) {
        case RECTANGULAR:
            integrateRectangular(cmd,cycle_time);
            break;
        default:
            throw std::runtime_error("Invalid integration Method: " + std::to_string(method));
        }
    }
    else{
         for(uint i = 0; i < cmd.size(); i++){
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
         initialized = true;
    }
    prev_cmd = cmd;
}

void JointIntegrator::integrateRectangular(base::commands::Joints &cmd, double cycle_time){
    for(uint i = 0; i < cmd.size(); i++)
    {
        switch(cmdMode(cmd[i]))
        {
        case base::JointState::SPEED:
            cmd[i].position = prev_cmd[i].position + cmd[i].speed*cycle_time;
            break;
        case base::JointState::ACCELERATION:
            cmd[i].speed = prev_cmd[i].speed + cmd[i].acceleration*cycle_time;
            cmd[i].position = prev_cmd[i].position + cmd[i].speed*cycle_time;
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
