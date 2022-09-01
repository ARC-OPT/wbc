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

    base::samples::RigidBodyStateSE3 ref_frame;

    // Create equation system
    //    Walk through all priorities and update the optimization problem. The outcome will be
    //    A - Vector of task matrices. One matrix for each priority
    //    y - Vector of task velocities. One vector for each priority
    //    W - Vector of task weights. One vector for each priority

    int prio = 0; // Only one priority is implemented here!
    tasks_prio[prio].resize(robot_model->noOfJoints(), n_task_variables_per_prio[prio], 0, false);

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
        tasks_prio[prio].Wy.segment(row_index, n_vars) = task->weights_root * task->activation * (!task->timeout);
        tasks_prio[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = task->A;
        tasks_prio[prio].lower_y.segment(row_index, n_vars) = task->y_ref_root;
        tasks_prio[prio].upper_y.segment(row_index, n_vars) = task->y_ref_root;

        row_index += n_vars;
    }
    const base::MatrixXd& A = tasks_prio[prio].A;
    const base::VectorXd& y = tasks_prio[prio].lower_y;

    // Cost Function: x^T*H*x + x^T * g
    tasks_prio[prio].H.noalias() = A.transpose() * A;
    tasks_prio[prio].g.setZero().noalias() = -y.transpose() * A;
    tasks_prio[prio].upper_x.setConstant(1000);
    tasks_prio[prio].lower_x.setConstant(-1000);

    tasks_prio[prio].A.setZero();
    tasks_prio[prio].lower_y.setZero();
    tasks_prio[prio].upper_y.setZero();

    tasks_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all tasks!?
    tasks_prio.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    return tasks_prio;
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
