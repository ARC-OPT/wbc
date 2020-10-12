#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

enum ModelType{
    UNSET = 0,
    ROBOT = 1,
    VIRTUAL_6_DOF_JOINT = 2
};

/**
 * @brief Robot Model configuration class
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        initial_state.pose.fromTransform(Eigen::Affine3d::Identity());
        initial_state.twist.setZero();
        initial_state.acceleration.setZero();
        type = ModelType::UNSET;
    }
    RobotModelConfig(const std::string& file,
                     const std::vector<std::string> joint_names,
                     const std::vector<std::string> actuated_joint_names,
                     const ModelType& type,
                     const std::string& hook = "",
                     const base::samples::RigidBodyStateSE3& initial_state = base::samples::RigidBodyStateSE3()) :
        file(file),
        joint_names(joint_names),
        actuated_joint_names(actuated_joint_names),
        type(type),
        hook(hook),
        initial_state(initial_state){
    }

    std::string file;                               /** Path to robot model file*/
    std::vector<std::string> joint_names;           /** Set the order of joint names here..*/
    std::vector<std::string> actuated_joint_names;  /** Set the order of actuated joint names here.*/
    ModelType type;                                 /** The type of model, see ModelType*/
    std::string hook;                               /** Optional: In case of multiple models, define the Link to which this robot model is attached in the overall model*/
    base::samples::RigidBodyStateSE3 initial_state; /** Optional: In case of multiple models, define the initial state of this model*/
};

}
#endif // ROBOTMODELCONFIG_HPP
