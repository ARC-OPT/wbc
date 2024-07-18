#include "RobotModel.hpp"
#include <fstream>
#include <urdf_parser/urdf_parser.h>

namespace wbc{

RobotModel::RobotModel() :
    gravity(Eigen::Vector3d(0,0,-9.81)), configured(false), updated(false){
}

void RobotModel::update(const Eigen::VectorXd& joint_positions,
                        const Eigen::VectorXd& joint_velocities){
    assert(!hasFloatingBase()); // This method is for fixed base robots only!
    zero_jnt.setZero(na());
    update(joint_positions, joint_velocities, zero_jnt);
}

void RobotModel::update(const Eigen::VectorXd& joint_positions,
                        const Eigen::VectorXd& joint_velocities,
                        const Eigen::VectorXd& joint_accelerations){

    assert(!hasFloatingBase()); // This method is for fixed base robots only!
    update(joint_positions, joint_velocities, joint_accelerations, types::Pose(), types::Twist(), types::SpatialAcceleration());
}

void RobotModel::update(const Eigen::VectorXd& joint_positions,
                        const Eigen::VectorXd& joint_velocities,
                        const types::Pose& fb_pose,
                        const types::Twist& fb_twist){
    assert(hasFloatingBase()); // This method is for floating base robots only!
    zero_jnt.setZero(na());
    zero_acc.setZero();
    update(joint_positions, joint_velocities, zero_jnt, fb_pose, fb_twist, zero_acc);
}

void RobotModel::clear(){
    independent_joint_names.clear();
    joint_names.clear();
    actuated_joint_names.clear();
    base_frame="";
    world_frame="";
    gravity = Eigen::Vector3d(0,0,-9.81);
    has_floating_base = false;
    joint_limits.clear();
    robot_urdf.reset();
    joint_state.clear();
    configured = false;
    updated = false;
}

void RobotModel::setContacts(const std::vector<Contact> &_contacts){
    contacts = _contacts;
}


uint RobotModel::jointIndex(const std::string &joint_name){
    assert(configured);
    uint idx = std::find(joint_names.begin(), joint_names.end(), joint_name) - joint_names.begin();
    if(idx >= joint_names.size())
        return -1;
    return idx;
}


bool RobotModel::hasLink(const std::string &link_name){
    assert(configured);
    if(link_name == "world" && hasFloatingBase())
        return true;
    for(auto l  : robot_urdf->links_)
        if(l.second->name == link_name)
            return true;
    return false;
}

bool RobotModel::hasJoint(const std::string &joint_name){
    assert(configured);
    return std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end();
}

bool RobotModel::hasActuatedJoint(const std::string &joint_name){
    assert(configured);
    return std::find(actuated_joint_names.begin(), actuated_joint_names.end(), joint_name) != actuated_joint_names.end();
}

urdf::ModelInterfaceSharedPtr RobotModel::loadRobotURDF(const std::string& file_or_string){
    std::ifstream fs(file_or_string.c_str());
    if(fs)
        return urdf::parseURDFFile(file_or_string);
    else
        return urdf::parseURDF(file_or_string);
}

/** Return number of active contacts*/
uint RobotModel::nac(){
    assert(configured);
    uint n = 0;
    for(auto a : contacts){
        if(a.active)
            n++;
    }
    return n;
}

RobotModelFactory::RobotModelMap* RobotModelFactory::robot_model_map = 0;

} // namespace wbc
