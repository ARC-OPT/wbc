#include "RobotModel.hpp"

namespace wbc{

RobotModel::RobotModel(){

}

RobotModel::~RobotModel(){

}

uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(all_joint_names.begin(), all_joint_names.end(), joint_name) - all_joint_names.begin();
    if(idx >= all_joint_names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}

const std::string& RobotModel::baseFrame(){
    return base_frame;
}

} // namespace wbc
