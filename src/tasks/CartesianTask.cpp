#include "CartesianTask.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc {

CartesianTask::CartesianTask(const TaskConfig &_config, uint n_robot_joints) :
    Task(_config, n_robot_joints){

}

CartesianTask::~CartesianTask(){

}

} //namespace wbc
