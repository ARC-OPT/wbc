#include "RobotModel.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

RobotModel::RobotModel() :
    gravity(base::Vector3d(0,0,-9.81)){
}

void RobotModel::updateFloatingBase(const base::RigidBodyStateSE3& rbs,
                                    const std::vector<std::string> &floating_base_virtual_joint_names,
                                    base::samples::Joints& joint_state){

    if(floating_base_virtual_joint_names.size() != 6){
        LOG_ERROR("Size of floating base virtual joint names has to be 6 but is %i", floating_base_virtual_joint_names.size());
        throw std::runtime_error("Invalid floating base virtual joint names");
    }

    if(!rbs.hasValidPose() ||
       !rbs.hasValidTwist() ||
       !rbs.hasValidAcceleration()){
       LOG_ERROR("Invalid status of floating base given! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion)");
       throw std::runtime_error("Invalid floating base status");
    }
    base::JointState js;
    base::Vector3d euler = rbs.pose.toTransform().rotation().eulerAngles(0,1,2); // TODO: Use Rotation Vector instead?
    for(int j = 0; j < 3; j++){
        js.position = rbs.pose.position(j);
        js.speed = rbs.twist.linear(j);
        js.acceleration = rbs.acceleration.linear(j);
        joint_state[floating_base_virtual_joint_names[j]] = js;

        js.position = euler(j);
        js.speed = rbs.twist.angular(j);
        js.acceleration = rbs.acceleration.angular(j);
        joint_state[floating_base_virtual_joint_names[j+3]] = js;
    }
}
/*
bool RobotModel::configure(const RobotModelConfig& cfg){

    robot_urdf = urdf::parseURDFFile(cfg.file);
    if(!robot_urdf){
        LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
        return false;
    }

    // Blacklist not required joints
    URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist);

    actuated_joint_names = cfg.actuated_joint_names;

    // Add floating base
    has_floating_base = cfg.floating_base;
    if(has_floating_base)
        floating_base_names = URDFTools::addFloatingBaseToURDF(robot_urdf, cfg.world_frame_id);

    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits);

    // Check if joint names and joint names in URDF are consistent
    current_joint_state.elements.resize(cfg.joint_names.size());
    current_joint_state.names = cfg.joint_names;
    for(auto n : floating_base_names)
        if(!hasJoint(n)){
            LOG_ERROR_S << "If you set 'floating_base' to 'true', you have to add the following virtual joints to joint_names: "<<std::endl;
            LOG_ERROR_S << "   floating_base_trans_x, floating_base_trans_y, floating_base_trans_z" << std::endl;
            LOG_ERROR_S << "   floating_base_rot_x,   floating_base_rot_y,   floating_base_rot_z  " << std::endl;
            return false;
        }
    const std::vector<std::string> joint_names_urdf = URDFTools::jointNamesFromURDF(robot_urdf);
    for(auto n : joint_names_urdf)
        if(!hasJoint(n)){
            LOG_ERROR_S << "Joint " << n << " is a non-fixed joint in the URDF model, but it has not been configured in joint_names" << std::endl;
            return false;
        }

    // Set initial floating base state
    if(has_floating_base){
        if(cfg.floating_base_state.hasValidPose() &&
           cfg.floating_base_state.hasValidTwist() &&
           cfg.floating_base_state.hasValidAcceleration())
            updateFloatingBase(cfg.floating_base_state, current_joint_state);
    }

    base_frame =  robot_urdf->getRoot()->name;
    contact_points = cfg.contact_points;
    for(auto c : contact_points){
        if(!hasLink(c)){
            LOG_ERROR("Contact point %s is not a valid link in the robot model", c.c_str());
            return false;
        }
    }
    joint_space_inertia_mat.resize(noOfJoints(), noOfJoints());
    bias_forces.resize(noOfJoints());

    selection_matrix.resize(noOfActuatedJoints(),noOfJoints());
    selection_matrix.setZero();
    for(int i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, jointIndex(actuated_joint_names[i])) = 1.0;

    if(getenv("BASE_LOG_LEVEL")){
        if(std::string(getenv("BASE_LOG_LEVEL")) == "INFO" || std::string(getenv("BASE_LOG_LEVEL")) == "DEBUG"){
            std::cout<<"Actuated Joints: "<<std::endl;
            for(auto n : actuatedJointNames())
                std::cout<<n;
            std::cout<<std::endl;

            std::cout<<"All Joints: "<<std::endl;
            for(auto n : jointNames())
                std::cout << n;
            std::cout<<std::endl;

            std::cout<<"URDF Tree"<<std::endl;
            URDFTools::printTree(robot_urdf->getRoot());
        }
    }
    return true;
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

uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) - current_joint_state.names.begin();
    if(idx >= current_joint_state.names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}

bool RobotModel::hasLink(const std::string &link_name){
    for(auto l  : robot_urdf->links_)
        if(l.second->name == link_name)
            return true;
    return false;
}

bool RobotModel::hasJoint(const std::string &joint_name){
    return std::find(current_joint_state.names.begin(), current_joint_state.names.end(), joint_name) != current_joint_state.names.end();
}

bool RobotModel::hasActuatedJoint(const std::string &joint_name){
    return std::find(actuated_joint_names.begin(), actuated_joint_names.end(), joint_name) != actuated_joint_names.end();
}
*/
} // namespace wbc
