#include "JointIntegrator.hpp"
#include "Logger.hpp"

namespace wbc {

using namespace types;

void JointIntegrator::integrate(const JointState& joint_state, JointCommand &cmd, double cycle_time, CommandMode mode, IntegrationMethod method, bool use_cur_state){

    cur_joint_state = joint_state;
    if(initialized){
        switch (method) {
        case RECTANGULAR:
            integrateRectangular(cmd,cycle_time,mode,use_cur_state);
            break;
        case TRAPEZOIDAL:
            integrateTrapezoidal(cmd,cycle_time,mode,use_cur_state);
            break;
        default:
            log(logERROR) << "Invalid integration method " << method;
            assert(method == RECTANGULAR || method == TRAPEZOIDAL);
            break;
        }
    }
    else{
        switch(mode)
        {
        case VELOCITY:{
            cmd.position = joint_state.position;
            break;
        }
        case ACCELERATION:{
            cmd.position = joint_state.position;
            cmd.velocity = joint_state.velocity;
            break;
        }
        default:
            log(logERROR) << "Invalid integration method " << method;
            assert(method == RECTANGULAR || method == TRAPEZOIDAL);
            break;
        }
        initialized = true;
    }
    prev_cmd = cmd;
}

void JointIntegrator::integrateRectangular(JointCommand &cmd, double cycle_time, CommandMode mode, bool use_cur_state){

    if(use_cur_state){
        prev_cmd.position = cur_joint_state.position;
        prev_cmd.velocity = cur_joint_state.velocity;
        prev_cmd.acceleration = cur_joint_state.acceleration;
    }

    switch(mode){
    case VELOCITY:{
        cmd.position = prev_cmd.position + cmd.velocity*cycle_time;
        break;
    }
    case ACCELERATION:{
        cmd.velocity = prev_cmd.velocity + cmd.acceleration*cycle_time;
        cmd.position = prev_cmd.position + cmd.velocity*cycle_time;
        break;
    }
    default:
        log(logERROR) << "Invalid control mode " << mode;
        assert(mode == VELOCITY || mode == ACCELERATION);
        break;
    }
}

void JointIntegrator::integrateTrapezoidal(JointCommand &cmd, double cycle_time, CommandMode mode, bool use_cur_state){

    if(use_cur_state){
        prev_cmd.position = cur_joint_state.position;
        prev_cmd.velocity = cur_joint_state.velocity;
        prev_cmd.acceleration = cur_joint_state.acceleration;
    }

    switch(mode){
    case VELOCITY:{
        cmd.position = prev_cmd.position + (cmd.velocity+prev_cmd.velocity)*cycle_time/2;
        break;
    }
    case ACCELERATION:{
        cmd.velocity = prev_cmd.velocity + (cmd.acceleration+prev_cmd.acceleration)*cycle_time/2;
        cmd.position = prev_cmd.position + (cmd.velocity+prev_cmd.velocity)*cycle_time/2;
        break;
    }
    default:
        log(logERROR) << "Invalid control mode " << mode;
        assert(mode == VELOCITY || mode == ACCELERATION);
        break;
    }
}
}
