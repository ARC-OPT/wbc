#ifndef SUBTASK_CPP
#define SUBTASK_CPP

#include "SubTask.hpp"

namespace wbc{

void SubTask::setReference(const base::samples::RigidBodyState &ref){
    if(config.type != task_type_cartesian){
        LOG_ERROR("Reference input of task %s is Cartesian but task is not Cartesian", config.name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(!ref.hasValidVelocity() ||
       !ref.hasValidAngularVelocity()){
        LOG_ERROR("Reference input of task %s has invalid velocity and/or angular velocity", config.name.c_str());
        throw std::invalid_argument("Invalid Cartesian reference input");
    }

    y_des.segment(0,3) = ref.velocity;
    y_des.segment(3,3) = ref.angular_velocity;

    updateTime();
}

void SubTask::setReference(const base::samples::Joints& ref){
    if(config.type != task_type_joint){
        LOG_ERROR("Reference input of task %s is in joint space but task is not in joint space", config.name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(ref.size() != no_task_vars){
        LOG_ERROR("Size for input reference of task %s should be %i but is %i", config.name.c_str(), no_task_vars, ref.size());
        throw std::invalid_argument("Invalid joint reference input");
    }

    for(uint i = 0; i < config.task_var_names.size(); i++){
        if(!ref[i].hasSpeed()){
            LOG_ERROR("Reference input for joint %s of task %s has invalid speed value(s)", ref.names[i].c_str(), config.name.c_str());
            throw std::invalid_argument("Invalid joint reference input");
        }
        y_des(i) = ref[i].speed;
    }

    updateTime();
}

void SubTask::updateTime(){
    base::Time now = base::Time::now();
    update_timediff = (now - last_task_input).toSeconds();
    last_task_input = now;
    double cur_rate = 1/update_timediff;
    avg_update_rate = avg_update_rate + 0.1 * (cur_rate - avg_update_rate);
}

void SubTask::validate()
{
    if(activation < 0 || activation > 1){
        LOG_ERROR("Sub Task: %s. Activation must be >= 0 and <= 1, but is %f", config.name.c_str(), activation);
        throw std::invalid_argument("Invalid activation value");
    }
    if(weights.size() !=  no_task_vars){
        LOG_ERROR("Size of weight vector is %f, but task %s has %f task variables", weights.size(), config.name.c_str(), no_task_vars);
        throw std::invalid_argument("Invalid no of task weights");
    }
}

void SubTask::computeDebug(const base::VectorXd& solver_output, const base::VectorXd& robot_vel){
    y_solution = A * solver_output;
    y = A * robot_vel;
    sqrt_ctrl_err = sqrt_wbc_err = 0;
    for(uint i = 0; i < no_task_vars; i++)
    {
        wbc_error(i) = fabs(y_des(i) - y_solution(i));
        ctrl_error(i) = fabs(y_des(i) - y(i));
        sqrt_wbc_err += wbc_error(i);
        sqrt_ctrl_err += ctrl_error(i);
    }
}

void SubTask::reset()
{
    y_des.setZero();
    y_des_root_frame.setZero();
    y_solution.setZero();
    wbc_error.setZero();
    y.setZero();
    ctrl_error.setZero();
    weights = base::VectorXd::Ones(no_task_vars);
    if(tasks_initially_active)
        activation = 1;
    else
        activation = 0;
    task_timed_out = 0;
    sqrt_wbc_err = 0;
    sqrt_ctrl_err = 0;

    A.setZero();
    last_task_input = base::Time::now();
    update_timediff = 0;
    avg_update_rate = 0;
}
} //namespace wbc
#endif // SUBTASK_CPP
