#include "Task.hpp"

namespace wbc{

Task::Task(){

}

Task::Task(TaskConfig config, uint nv, uint nj, TaskType type) :
    config(config), nv(nv), nj(nj), type(type){

    y_ref.resize(nv);
    y_ref_world.resize(nv);
    weights.resize(nv);
    weights_world.resize(nv);

    A.resize(nv, nj);
    Aw.resize(nv, nj);
    reset();

    setWeights(Eigen::Map<Eigen::VectorXd>(config.weights.data(),config.weights.size()));
    setActivation(config.activation);
}

Task::~Task(){

}

void Task::reset(){
    y_ref_world.setConstant(nv, std::numeric_limits<double>::quiet_NaN());
    y_ref.setZero(nv);
    A.setZero();
    Aw.setZero();
    activation = 0;
    for(uint i = 0; i < nv; i++){
        weights(i) = 1.0;
        weights_world(i) = 1.0;
    }
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
