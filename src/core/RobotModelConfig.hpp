#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/NamedVector.hpp>

namespace wbc{

struct ActiveContacts : public base::NamedVector<int>{
};

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
        type = "kdl";
    }
    RobotModelConfig(const std::string& file,
                     const std::vector<std::string> joint_names = {},
                     const std::vector<std::string> actuated_joint_names = {},
                     const bool floating_base = false,
                     const std::string &world_frame_id = "world",
                     const base::RigidBodyStateSE3& floating_base_state = base::RigidBodyStateSE3(),
                     const ActiveContacts &contact_points = ActiveContacts(),
                     const std::string& submechanism_file = "",
                     const std::vector<std::string>& joint_blacklist = std::vector<std::string>()) :
        file(file),
        submechanism_file(submechanism_file),
        type("kdl"),
        joint_names(joint_names),
        actuated_joint_names(actuated_joint_names),
        floating_base(floating_base),
        world_frame_id(world_frame_id),
        floating_base_state(floating_base_state),
        contact_points(contact_points),
        joint_blacklist(joint_blacklist){

    }

    /** Absolute path to URDF file describing the robot model.*/
    std::string file;
    /** Only Hyrodyn models: Absolute path to submechanism file, which describes the kinematic structure including parallel mechanisms.*/
    std::string submechanism_file;
    /** Model type. Must be the exact name of one of the registered robot model plugins. See src/robot_models for all available plugins. Default is kdl*/
    std::string type;
    /** Optional: Order of joints used internally. If left empty, joint order will be alphabetical (from URDF parser). If not empty, this has to contain all non-fixed joints from URDF model. If floating_base is true,
      * additionally, the first 6 joint names have to be {floating_base_trans_x, floating_base_trans_y, floating_base_trans_z, floating_base_rot_x, floating_base_rot_y, floating_base_rot_z}.
      * The order here will be used in all computed quantities, like Jacobians, control output, etc.. Note: For Hyrodyn robot models, the joint order will be defined in the submechanism files, so this property will be ignored*/
    std::vector<std::string> joint_names;
    /** Optional: Define actuated joints here. If empty, joint_names will be used as actuated_joints, i.e., all joints are actuated.
      * Note: For Hyrodyn robot models, the actuated joints will be defined in the submechanism files, so this property will be ignored*/
    std::vector<std::string> actuated_joint_names;
    /** Optional: Attach a virtual 6 DoF floating base to the model: Naming scheme of the joints is currently fixed:
      * floating_base_trans_x, floating_base_trans_y, floating_base_trans_z,
      * floating_base_rot_x, floating_base_rot_y, floating_base_rot_z*/
    bool floating_base;
    /** Optional, only if floating_base is set to true: ID of the world frame, defaults to 'world'*/
    std::string world_frame_id;
    /** Optional, only if floating_base is set to true: Initial state of the floating base. A valid pose has to be given, twist/acceleration is optional*/
    base::RigidBodyStateSE3 floating_base_state;
    /** Optional: Link names that are possibly in contact with the environment. These have to valid link names in the robot model.*/
    ActiveContacts contact_points;
    /** Optional: Blacklist some joint that shall not be used in the model. They will be replaced by fixed joints*/
    std::vector<std::string> joint_blacklist;
};

}
#endif // ROBOTMODELCONFIG_HPP
