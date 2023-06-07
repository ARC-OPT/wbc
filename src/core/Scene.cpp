#include "Scene.hpp"
#include <base-logging/Logging.hpp>
#include "../tasks/JointTask.hpp"
#include "../tasks/CartesianTask.hpp"

namespace wbc{

Scene::Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double dt) :
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
    tasks_status.clear();
    configured = false;
}

bool Scene::configure(const std::vector<TaskConfig> &config){

    solver->reset();
    clearTasks();
    if(config.empty()){
        LOG_ERROR("Empty WBC Task configuration");
        return false;
    }

    for(auto c : config)
        c.validate();
    std::vector< std::vector<TaskConfig> > sorted_config;
    sortTaskConfig(config, sorted_config);

    //// Create tasks. Store the number of task variables per priority
    ///
    tasks.resize(sorted_config.size());
    for(size_t i = 0; i < sorted_config.size(); i++){

        tasks[i].resize(sorted_config[i].size());
        for(size_t j = 0; j < sorted_config[i].size(); j++)
            tasks[i][j] = createTask(sorted_config[i][j]);
    }
    n_task_variables_per_prio = getNTaskVariablesPerPrio(config);

    for(size_t i = 0; i < tasks.size(); i++){
        for(size_t j = 0; j < tasks[i].size(); j++){
            TaskPtr task = tasks[i][j];
            tasks_status.names.push_back(task->config.name);
            tasks_status.elements.push_back(TaskStatus());
        }
    }

    hqp.resize(tasks.size());
    configured = true;

    // Set actuated joint weights to 1 and unactuated joint weight to 0 by default
    joint_weights.resize(robot_model->noOfJoints());
    joint_weights.names = robot_model->jointNames();
    std::fill(joint_weights.elements.begin(), joint_weights.elements.end(), 1);

    actuated_joint_weights.resize(robot_model->noOfActuatedJoints());
    actuated_joint_weights.names = robot_model->actuatedJointNames();
    std::fill(actuated_joint_weights.elements.begin(), actuated_joint_weights.elements.end(), 1);

    wbc_config = config;

    // Check WBC config
    for(auto cfg : wbc_config){
        if(cfg.type == cart){
            if(!robot_model->hasLink(cfg.root)){
                LOG_ERROR("Link %s is used in task config %s, but this link is not in robot model", cfg.root.c_str(), cfg.name.c_str());
                return false;
            }
            if(!robot_model->hasLink(cfg.tip)){
                LOG_ERROR("Link %s is used in task config %s, but this link is not in robot model", cfg.tip.c_str(), cfg.name.c_str());
                return false;
            }
            if(!robot_model->hasLink(cfg.ref_frame)){
                LOG_ERROR("Link %s is used in task config %s, but this link is not in robot model", cfg.ref_frame.c_str(), cfg.name.c_str());
                return false;
            }
        }

        try{
            cfg.validate();
        }
        catch(std::invalid_argument e){
            return false;
        }
    }

    return true;
}

void Scene::setReference(const std::string& constraint_name, const base::samples::Joints& ref){
    TaskPtr c = getTask(constraint_name);
    if(c->config.type == cart)
        throw std::runtime_error("Constraint '" + c->config.name + "' has type cart, but you are trying to set a joint space reference");
    std::static_pointer_cast<JointTask>(c)->setReference(ref);
}

void Scene::setReference(const std::string& constraint_name, const base::samples::RigidBodyStateSE3& ref){
    TaskPtr c = getTask(constraint_name);
    if(c->config.type == jnt)
        throw std::runtime_error("Constraint '" + c->config.name + "' has type jnt, but you are trying to set a cartesian reference");
    std::static_pointer_cast<CartesianTask>(getTask(constraint_name))->setReference(ref);
}

void Scene::setTaskWeights(const std::string& constraint_name, const base::VectorXd &weights){
    getTask(constraint_name)->setWeights(weights);
}

void Scene::setTaskActivation(const std::string& constraint_name, const double activation){
    getTask(constraint_name)->setActivation(activation);
}

TaskPtr Scene::getTask(const std::string& name){

    for(size_t i = 0; i < tasks.size(); i++){
        for(size_t j = 0; j < tasks[i].size(); j++){
            if(tasks[i][j]->config.name == name)
                return tasks[i][j];
        }
    }
    throw std::invalid_argument("Invalid constraint name: " + name);
}

bool Scene::hasTask(const std::string &name){

    for(size_t i = 0; i < tasks.size(); i++){
        for(size_t j = 0; j < tasks[i].size(); j++)
            if(tasks[i][j]->config.name == name)
                return true;
    }
    return false;
}

void Scene::sortTaskConfig(const std::vector<TaskConfig>& config, std::vector< std::vector<TaskConfig> >& sorted_config){

    // Get highest prio
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++){

        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    sorted_config.resize(max_prio + 1);

    for(uint i = 0; i < config.size(); i++)
        sorted_config[config[i].priority].push_back(config[i]);

    // Erase empty priorities
    for(uint i = 0; i < sorted_config.size(); i++){
        if(sorted_config[i].empty())
        {
            sorted_config.erase(sorted_config.begin() + i, sorted_config.begin() + i + 1);
            i--;
        }
    }
}

std::vector<int> Scene::getNTaskVariablesPerPrio(const std::vector<TaskConfig> &config){

    std::vector< std::vector<TaskConfig> > sorted_config;
    sortTaskConfig(config, sorted_config);

    std::vector<int> nn_pp(sorted_config.size());
    for(size_t i = 0; i < sorted_config.size(); i++){
        nn_pp[i] = 0;
        for(size_t j = 0; j < sorted_config[i].size(); j++)
            nn_pp[i] += sorted_config[i][j].nVariables();
    }
    return nn_pp;
}

void Scene::setJointWeights(const JointWeights &weights){

    if(weights.elements.size() != weights.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint weight vector do not match"<<std::endl;
        throw std::runtime_error("Invalid joint weights");
    }

    for(auto n : weights.names){
        try{
            joint_weights[n] = weights[n];
        }
        catch(base::NamedVector<double>::InvalidName e){
            LOG_ERROR_S<<"Joint name "<<n<<" is given in joint weight vector, but this joint is not in robot model"<<std::endl;
            std::cout<<"Joint names are "<<std::endl;
            for(auto n : joint_weights.names)
                std::cout<<n<<std::endl;

            throw e;
        }
    }

    for(auto n : actuated_joint_weights.names)
        actuated_joint_weights[n] = joint_weights[n];
}

SceneFactory::SceneMap* SceneFactory::scene_map = 0;


} // namespace wbc
