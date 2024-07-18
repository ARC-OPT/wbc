#include "Scene.hpp"
#include "Task.hpp"
#include "../tools/Logger.hpp"

loglevel_e loglevel = logWARNING;

namespace wbc{

Scene::Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double /*dt*/) :
    robot_model(robot_model),
    solver(solver),
    configured(false){


}

Scene::~Scene(){
}

void Scene::clearTasks(){

    for(uint i = 0; i < tasks.size(); i++ ){
        for(uint j = 0; j < tasks[i].size(); j++)
            tasks[i][j].reset();
        tasks[i].clear();
    }
    tasks.clear();
    configured = false;
}

bool Scene::configure(const std::vector<TaskPtr> &tasks_in){

    solver->reset();
    clearTasks();
    if(tasks_in.empty()){
        log(logERROR)<<"Empty WBC Task configuration";
        return false;
    }

    sortTasks(tasks_in, tasks);

    hqp.resize(tasks.size());
    configured = true;

    joint_weights.resize(robot_model->nj());
    joint_weights.setConstant(1.0);

    actuated_joint_weights.resize(robot_model->na());
    actuated_joint_weights.setConstant(1.0);

    return true;
}

TaskPtr Scene::getTask(const std::string& name){

    for(size_t i = 0; i < tasks.size(); i++){
        for(size_t j = 0; j < tasks[i].size(); j++){
            if(tasks[i][j]->config.name == name)
                return tasks[i][j];
        }
    }
    log(logERROR) << "Task with name " << name << " has not been configured";
    return TaskPtr();
}

bool Scene::hasTask(const std::string &name){

    for(size_t i = 0; i < tasks.size(); i++){
        for(size_t j = 0; j < tasks[i].size(); j++)
            if(tasks[i][j]->config.name == name)
                return true;
    }
    return false;
}

void Scene::sortTasks(const std::vector<TaskPtr>& tasks, std::vector< std::vector<TaskPtr> >& sorted_tasks){

    // Get highest prio
    int max_prio = 0;
    for(uint i = 0; i < tasks.size(); i++){

        if(tasks[i]->config.priority > max_prio)
            max_prio = tasks[i]->config.priority;
    }
    sorted_tasks.resize(max_prio + 1);

    for(uint i = 0; i < tasks.size(); i++)
        sorted_tasks[tasks[i]->config.priority].push_back(tasks[i]);

    // Erase empty priorities
    for(uint i = 0; i < sorted_tasks.size(); i++){
        if(sorted_tasks[i].empty())
        {
            sorted_tasks.erase(sorted_tasks.begin() + i, sorted_tasks.begin() + i + 1);
            i--;
        }
    }
}

uint Scene::getNTaskVariables(const std::vector<TaskPtr> &tasks){
    uint n_task_variables = 0;
    for(const TaskPtr& t : tasks)
        n_task_variables += t->nv;
    return n_task_variables;
}

std::vector<int> Scene::getNTaskVariablesPerPrio(const std::vector<TaskPtr> &tasks){

    std::vector< std::vector<TaskPtr> > tmp_tasks;
    sortTasks(tasks, tmp_tasks);

    std::vector<int> nn_pp(tmp_tasks.size());
    for(size_t i = 0; i < tmp_tasks.size(); i++){
        nn_pp[i] = 0;
        for(size_t j = 0; j < tmp_tasks[i].size(); j++)
            nn_pp[i] += tmp_tasks[i][j]->nv;
    }
    return nn_pp;
}

void Scene::setJointWeights(const Eigen::VectorXd &weights){
    assert(weights.size() == joint_weights.size());
    joint_weights = weights;
}

SceneFactory::SceneMap* SceneFactory::scene_map = 0;


} // namespace wbc
