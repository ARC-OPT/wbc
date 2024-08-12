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
    space_jac_map.clear();
    body_jac_map.clear();
    pose_map.clear();
    twist_map.clear();
    acc_map.clear();
    spatial_acc_bias_map.clear();
    joint_space_inertia_mat.clear();
    bias_forces.clear();
    com_jac.clear();
    com_rbs.clear();
}

void RobotModel::resetData(){
    for(auto &it : joint_space_inertia_mat)
        it.needs_recompute = true;
    for(auto &it : com_jac)
        it.needs_recompute = true;
    for(auto &it : bias_forces)
        it.needs_recompute = true;
    for(auto &it : com_rbs)
        it.needs_recompute = true;
    for(auto &it : space_jac_map)
        it.second.needs_recompute = true;
    for(auto &it : body_jac_map)
        it.second.needs_recompute = true;
    for(auto &it : pose_map)
        it.second.needs_recompute = true;
    for(auto &it : twist_map)
        it.second.needs_recompute = true;
    for(auto &it : acc_map)
        it.second.needs_recompute = true;
    for(auto &it : spatial_acc_bias_map)
        it.second.needs_recompute = true;
    fk_needs_recompute = fk_with_zero_acc_needs_recompute = true;
}

void RobotModel::updateData(){
    for(auto it : joint_space_inertia_mat)
        it.data = jointSpaceInertiaMatrix();
    for(auto it : com_jac)
        it.data = comJacobian();
    for(auto it : bias_forces)
        it.data = biasForces();
    for(auto it : com_rbs)
        it.data = centerOfMass();
    for(auto it : space_jac_map)
        it.second.data = spaceJacobian(it.first);
    for(auto it : body_jac_map)
        it.second.data = bodyJacobian(it.first);
    for(auto it : pose_map)
        it.second.data = pose(it.first);
    for(auto it : twist_map)
        it.second.data = twist(it.first);
    for(auto it : acc_map)
        it.second.data = acceleration(it.first);
    for(auto it : spatial_acc_bias_map)
        it.second.data = spatialAccelerationBias(it.first);
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
