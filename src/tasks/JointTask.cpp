#include "JointTask.hpp"

namespace wbc {

JointTask::JointTask(const TaskConfig& _config, uint n_robot_joints) :
    Task(_config, n_robot_joints){

}

JointTask::~JointTask(){

}

} //namespace wbc
