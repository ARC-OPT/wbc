#include "RobotModel.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

std::vector<std::string> operator+(std::vector<std::string> a, std::vector<std::string> b){
    a.insert( a.end(), b.begin(), b.end() );
    return a;
}

RobotModel::RobotModel() :
    gravity(base::Vector3d(0,0,-9.81)){
}

void RobotModel::clear(){
    independent_joint_names.clear();
    joint_names.clear();
    actuated_joint_names.clear();
    joint_names_floating_base.clear();
    contact_points.clear();
    base_frame="";
    world_frame="";
    gravity = base::Vector3d(0,0,-9.81);
    has_floating_base = false;
    joint_limits.clear();
    robot_urdf.reset();
    joint_state.clear();
    joint_state_out.clear();
    space_jac_map.clear();
    body_jac_map.clear();
    jac_dot_map.clear();
}

void RobotModel::updateFloatingBase(const base::samples::RigidBodyStateSE3& rbs,
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

    floating_base_state = rbs;
    base::JointState js;
    base::Vector3d euler = rbs.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
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

    // Set timestamp of joint state vector to floating base timestamp in case it is older
    if(rbs.time.isNull()){
        LOG_ERROR("Floating base state does not have a valid timestamp. Or do we have 1970?");
        throw std::runtime_error("Invalid call to update()");
    }

    if(rbs.time < joint_state.time)
        joint_state.time = rbs.time;
}

void RobotModel::setActiveContacts(const ActiveContacts &contacts){
    for(auto name : contacts.names){
        if(contacts[name] != 0 && contacts[name] != 1)
            throw std::runtime_error("RobotModel::setActiveContacts: Contact value has to been 0 or 1");
    }
    active_contacts = contacts;
}


uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(joint_names.begin(), joint_names.end(), joint_name) - joint_names.begin();
    if(idx >= joint_names.size())
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
    return std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end();
}

bool RobotModel::hasActuatedJoint(const std::string &joint_name){
    return std::find(actuated_joint_names.begin(), actuated_joint_names.end(), joint_name) != actuated_joint_names.end();
}

const base::samples::Joints& RobotModel::jointState(const std::vector<std::string> &joint_names){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModel: You have to call update() with appropriately timestamped joint data at least once before requesting joint state information!");
        throw std::runtime_error("Invalid call to jointState()");
    }

    joint_state_out.resize(joint_names.size());
    joint_state_out.names = joint_names;
    joint_state_out.time = joint_state.time;

    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            joint_state_out[i] = joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("RobotModel: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state_out;
}

} // namespace wbc
