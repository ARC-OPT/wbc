#include "VelocityScene.hpp"
#include "../core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

#include "../tasks/JointVelocityTask.hpp"
#include "../tasks/CartesianVelocityTask.hpp"
#include "../tasks/CoMVelocityTask.hpp"

namespace wbc{

TaskPtr VelocityScene::createTask(const TaskConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianVelocityTask>(config, robot_model->noOfJoints());
    else if(config.type == com)
        return std::make_shared<CoMVelocityTask>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointVelocityTask>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Task with name %s has an invalid task type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid task config");
    }
}

const HierarchicalQP& VelocityScene::update(){

    if(!configured)
        throw std::runtime_error("VelocityScene has not been configured!. PLease call configure() before calling update() for the first time!");

    ///////// Tasks

    // Note: This scene models all tasks as linear equality constraints in order to comply with the HLS solver
    uint nj = robot_model->noOfJoints();
    for(uint prio = 0; prio < tasks.size(); prio++){

        uint nc = n_task_variables_per_prio[prio];
        hqp[prio].resize(nj, nc, 0, false);

        // Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < tasks[prio].size(); i++){

            TaskPtr task = tasks[prio][i];

            task->checkTimeout();
            task->update(robot_model);
            
            uint n_vars = task->config.nVariables();

            // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
            // task phases and we don't want to store the "old" reference value, in case we switch on the task again
            if(task->activation == 0){
               task->y_ref.setZero();
               task->y_ref_root.setZero();
            }

            // Insert tasks into equation system of current priority at the correct position. Note: Weights will be zero if activations
            // for this task is zero or if the task is in timeout
            hqp[prio].Wy.segment(row_index, n_vars) = task->weights_root * task->activation * (!task->timeout);
            hqp[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = task->A;
            hqp[prio].b.segment(row_index, n_vars) = task->y_ref_root;
            hqp[prio].H.setIdentity();
            hqp[prio].lower_x.resize(0);
            hqp[prio].upper_x.resize(0);
            hqp[prio].g.setZero();

            row_index += n_vars;

        } // tasks on prio
    } // priorities

    hqp.time = base::Time::now(); //  TODO: Use latest time stamp from all tasks!?

    // Joint Weights
    hqp.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    return hqp;
}

const base::commands::Joints& VelocityScene::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    // Convert Output
    solver_output_joints.resize(robot_model->noOfActuatedJoints());
    solver_output_joints.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        uint idx = robot_model->jointIndex(name);
        if(base::isNaN(solver_output[idx])){
            hqp[0].print();
            throw std::runtime_error("Solver output (speed) for joint " + name + " is NaN");
        }
        solver_output_joints[name].speed = solver_output[idx];
    }

    solver_output_joints.time = base::Time::now();
    return solver_output_joints;
}

const TasksStatus& VelocityScene::updateTasksStatus(){

    robot_model->systemState(q,qd,qdd);

    for(uint prio = 0; prio < tasks.size(); prio++){
        for(uint i = 0; i < tasks[prio].size(); i++){
            TaskPtr task = tasks[prio][i];
            const std::string &name = task->config.name;

            tasks_status[name].time       = task->time;
            tasks_status[name].config     = task->config;
            tasks_status[name].activation = task->activation;
            tasks_status[name].timeout    = task->timeout;
            tasks_status[name].weights    = task->weights;
            tasks_status[name].y_ref      = task->y_ref;
            tasks_status[name].y_solution = task->A * solver_output;
            tasks_status[name].y          = task->A * qd;
        }
    }

    hqp.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    return tasks_status;
}


} // namespace wbc
