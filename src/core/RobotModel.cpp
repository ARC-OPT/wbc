#include "RobotModel.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

RobotModel::RobotModel() :
    gravity(base::Vector3d(0,0,-9.81)){

}

RobotModel::~RobotModel(){

}

void RobotModel::clear(){
    has_floating_base = false;
    joint_limits.clear();
    current_joint_state.clear();
    jac_map.clear();
    jac_dot_map.clear();
}

uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) - current_joint_state.names.begin();
    if(idx >= current_joint_state.names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}

void RobotModel::update(const base::samples::Joints& joint_state, const base::samples::RigidBodyStateSE3& _floating_base_state){
    for(size_t i = 0; i < joint_state.size(); i++){
        const std::string& name = joint_state.names[i];
        std::size_t idx;
        try{
            idx = current_joint_state.mapNameToIndex(name);
        }
        catch(base::samples::Joints::InvalidName e){
            LOG_ERROR_S<<"Robot model contains joint "<<name<<" but this joint is not in joint state vector"<<std::endl;
            throw e;
        }
        current_joint_state[idx] = joint_state[i];
        current_joint_state[idx].acceleration = 0.0;
    }
    current_joint_state.time = joint_state.time;
    floating_base_state = _floating_base_state;
    if(has_floating_base)
        if(floating_base_state.time > current_joint_state.time)
            current_joint_state.time = floating_base_state.time;
}

const base::samples::Joints& RobotModel::jointState(const std::vector<std::string> &joint_names){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to jointState()");
    }

    joint_state_out.resize(joint_names.size());
    joint_state_out.names = joint_names;
    joint_state_out.time = current_joint_state.time;

    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            joint_state_out[i] = current_joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("RobotModelKDL: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state_out;
}

const base::MatrixXd &RobotModel::selectionMatrix(){
    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, jointIndex(actuated_joint_names[i])) = 1.0;
    return selection_matrix;

}

} // namespace wbc
