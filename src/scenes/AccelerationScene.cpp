#include "AccelerationScene.hpp"
#include "../core/RobotModel.hpp"
#include <base-logging/Logging.hpp>
#include "../tasks/JointAccelerationTask.hpp"
#include "../tasks/CartesianAccelerationTask.hpp"
#include "../tasks/CoMAccelerationTask.hpp"

namespace wbc{

TaskPtr AccelerationScene::createTask(const TaskConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianAccelerationTask>(config, robot_model->noOfJoints());
    else if(config.type == com)
        return std::make_shared<CoMAccelerationTask>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointAccelerationTask>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Task with name %s has an invalid task type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid task config");
    }
}

const HierarchicalQP& AccelerationScene::update(){

    if(!configured)
        throw std::runtime_error("AccelerationScene has not been configured!. PLease call configure() before calling update() for the first time!");

    if(tasks.size() != 1){
        LOG_ERROR("Number of priorities in AccelerationScene should be 1, but is %i", tasks.size());
        throw std::runtime_error("Invalid task configuration");
    }

    uint nv = robot_model->noOfJoints();
    uint nc = 0;

    int prio = 0; // Only one priority is implemented here!
    QuadraticProgram &qp = hqp[prio];
    qp.resize(nc, nv);

    ///////// Constraints

    // This scene does not implement constraints
    hqp[prio].upper_x.setConstant(1e10);
    hqp[prio].lower_x.setConstant(-1e10);

    hqp[prio].A.setZero();
    hqp[prio].lower_y.setZero();
    hqp[prio].upper_y.setZero();


    ///////// Tasks

    qp.H.setZero();
    qp.g.setZero();
    for(uint i = 0; i < tasks[prio].size(); i++){

        TaskPtr task = tasks[prio][i];

        task->checkTimeout();
        task->update(robot_model);

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0){
           task->y_ref.setZero();
           task->y_ref_root.setZero();
        }

        for(int i = 0; i < task->A.rows(); i++)
            task->Aw.row(i) = task->weights_root(i) * task->A.row(i) * task->activation * (!task->timeout);
        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = joint_weights[i] * task->Aw.col(i);

        qp.H += task->Aw.transpose()*task->Aw;
        qp.g -= task->Aw.transpose()*task->y_ref_root;
    }

    hqp.time = base::Time::now(); //  TODO: Use latest time stamp from all tasks!?
    hqp.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    return hqp;
}

const base::commands::Joints& AccelerationScene::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    // Convert Output
    solver_output_joints.resize(robot_model->noOfActuatedJoints());
    solver_output_joints.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        uint idx = robot_model->jointIndex(name);
        if(base::isNaN(solver_output[idx]))
            throw std::runtime_error("Solver output (acceleration) for joint " + name + " is NaN");
        solver_output_joints[name].acceleration = solver_output[idx];
    }
    solver_output_joints.time = base::Time::now();
    return solver_output_joints;
}

const TasksStatus &AccelerationScene::updateTasksStatus(){

    robot_acc.resize(robot_model->noOfJoints());
    uint nj = robot_model->noOfJoints();
    const base::samples::Joints &joint_state = robot_model->jointState(robot_model->jointNames());
    for(size_t i = 0; i < nj; i++)
        robot_acc(i) = joint_state[i].acceleration;

    for(uint prio = 0; prio < tasks.size(); prio++){
        for(uint i = 0; i < tasks[prio].size(); i++){
            TaskPtr task = tasks[prio][i];
            const std::string &name = task->config.name;

            tasks_status[name].time       = task->time;
            tasks_status[name].config     = task->config;
            tasks_status[name].activation = task->activation;
            tasks_status[name].timeout    = task->timeout;
            tasks_status[name].weights    = task->weights;
            tasks_status[name].y_ref      = task->y_ref_root;
            if(task->config.type == cart){
                const base::MatrixXd &jac = robot_model->spaceJacobian(task->config.root, task->config.tip);
                const base::Acceleration &bias_acc = robot_model->spatialAccelerationBias(task->config.root, task->config.tip);
                tasks_status[name].y_solution = jac * solver_output + bias_acc;
                tasks_status[name].y          = jac * robot_acc + bias_acc;
            }
        }
    }

    return tasks_status;
}

} // namespace wbc
