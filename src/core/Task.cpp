#include "Task.hpp"

namespace wbc{

Task::Task(){

}

Task::Task(TaskConfig config, RobotModelPtr robot_model, uint nv, TaskType type) :
    robot_model(robot_model), config(config), nv(nv), type(type){

    y_ref.resize(nv);
    weights.resize(nv);

    nj = robot_model->nj();

    A.resize(nv, nj);
    Aw.resize(nv, nj);
    reset();

    setWeights(Eigen::Map<Eigen::VectorXd>(config.weights.data(),config.weights.size()));
    setActivation(config.activation);
}

Task::~Task(){

}

void Task::reset(){
    y_ref.setZero(nv);
    A.setZero();
    Aw.setZero();
    setWeights(Eigen::Map<Eigen::VectorXd>(config.weights.data(),config.weights.size()));
    setActivation(config.activation);
}

void Task::setWeights(const Eigen::VectorXd& weights){
    assert(nv == weights.size());
    for(uint i = 0; i < nv; i++)
        assert(weights[i] >= 0);

    this->weights = weights;
}

void Task::setActivation(const double activation){
    assert(activation >= 0 && activation <= 1);
    this->activation = activation;
}

}// namespace wbc
