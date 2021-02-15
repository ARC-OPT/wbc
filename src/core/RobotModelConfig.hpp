#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

/**
 * @brief Robot Model configuration class
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        floating_base = false;
        world_frame_id = "world";
        floating_base_state.pose.position.setZero();
        floating_base_state.pose.orientation.setIdentity();
        floating_base_state.twist.setZero();
        floating_base_state.acceleration.setZero();
    }
    RobotModelConfig(const std::string& file,
                     const std::vector<std::string> joint_names,
                     const std::vector<std::string> actuated_joint_names,
                     const bool floating_base = false,
                     const std::string &world_frame_id = "world",
                     const base::RigidBodyStateSE3& floating_base_state = base::RigidBodyStateSE3(),
                     const std::vector<std::string> &contact_points = std::vector<std::string>(),
                     const std::string& submechanism_file = "") :
        file(file),
        submechanism_file(submechanism_file),
        joint_names(joint_names),
        actuated_joint_names(actuated_joint_names),
        floating_base(floating_base),
        world_frame_id(world_frame_id),
        floating_base_state(floating_base_state),
        contact_points(contact_points){

    }

    /** Absolute path to URDF file*/
    std::string file;
    /** Only Hyrodyn models: Absolute path to submechanism file*/
    std::string submechanism_file;
    /** Define all joints to use from the URDF model. The order here will be used in all computed quantities, like Jacobians, etc..*/
    std::vector<std::string> joint_names;
    /** Define all actuated joints here*/
    std::vector<std::string> actuated_joint_names;
    /** Optional: Attach a virtual 6 DoF floating base to the model: Naming scheme of the joints is currently fix:
      * floating_base_trans_x, floating_base_trans_y, floating_base_trans_z,
      * floating_base_rot_x, floating_base_rot_y, floating_base_rot_z*/
    bool floating_base;
    /** Optional, only if floating_base is set to true: ID of the world frame, defaults to 'world'*/
    std::string world_frame_id;
    /** Optional, only if floating_base is set to true: Initial state of the floating base. Pose defaults to identity, twist/acceleration to zero*/
    base::RigidBodyStateSE3 floating_base_state;
    /** Optional: Link names that are possibly in contact with the environment. These have to valid link names in the robot model.*/
    std::vector<std::string> contact_points;
};

}
#endif // ROBOTMODELCONFIG_HPP
