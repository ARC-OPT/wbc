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

    // Set timestamp of joint state vector to floating base timestamp in case it is older
    if(rbs.time.isNull()){
        LOG_ERROR("Floating base state does not have a valid timestamp. Or do we have 1970?");
        throw std::runtime_error("Invalid call to update()");
    }

    if(rbs.time < joint_state.time)
        joint_state.time = rbs.time;
}

} // namespace wbc
