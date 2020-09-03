#include "RobotModel.hpp"

namespace wbc{

RobotModel::RobotModel() :
    gravity(base::Vector3d(0,0,-9.81)){

}

RobotModel::~RobotModel(){

}

uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) - current_joint_state.names.begin();
    if(idx >= current_joint_state.names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}

const std::string& RobotModel::baseFrame(){
    return base_frame;
}

} // namespace wbc
