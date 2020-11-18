#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

enum ModelType{
    UNSET = 0,
    URDF = 1,        /** Standard URDF model file*/
    SUBMECHANISM = 2 /** For hyrodyn based robot models: yaml file defining the submechanisms*/
};

/**
 * @brief Robot Model configuration class
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        type = ModelType::UNSET;
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
                     const ModelType& type,
                     const bool floating_base = false,
                     const std::string &world_frame_id = "world",
                     const base::RigidBodyStateSE3& floating_base_state = base::RigidBodyStateSE3()
                     ) :
        file(file),
        joint_names(joint_names),
        actuated_joint_names(actuated_joint_names),
        type(type),
        floating_base(floating_base),
        world_frame_id(world_frame_id),
        floating_base_state(floating_base_state){

    }

    /** Absolute path to robot model file*/
    std::string file;
    /** Define all joints to use from the URDF model. The order here will be used in all computed quantities, like Jacobians, etc..*/
    std::vector<std::string> joint_names;
    /** Define all actuated joints here*/
    std::vector<std::string> actuated_joint_names;
    /** The type of model, see ModelType*/
    ModelType type;
    /** Only for model type URDF: Attach a virtual 6 DoF floating base to the model*/
    bool floating_base;
    /** Optional, only for model type URDF and only if floating_base is set to true: ID of the world frame, defaults to 'world'*/
    std::string world_frame_id;
    /** Optional, only for model type URDF and only if floating_base is set to true: Initial state of the floating base. Pose defaults to identity, twist/acceleration to zero*/
    base::RigidBodyStateSE3 floating_base_state;
};

}
#endif // ROBOTMODELCONFIG_HPP
