#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/Pose.hpp>
#include <base/NamedVector.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

/**
 * @brief Robot Model configuration class
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        initial_pose.fromTransform(Eigen::Affine3d::Identity());
    }
    RobotModelConfig(const std::string& _file,
                     const std::string& _hook = "",
                     const base::Pose& _initial_pose = base::Pose()) :
        file(_file),
        hook(_hook),
        initial_pose(_initial_pose){
    }

    std::string file;                      /** Path to robot model file*/
    std::vector<std::string> joint_names;  /** Set the order of joint names here (and define the joints that shall be used from the model, in case not all joints shall be used)*/
    std::string hook;                      /** Optional: In case of multiple models, define the Link to which this robot model is attached in the overall model*/
    base::Pose initial_pose;               /** Optional: In case of multiple models, define the initial pose of this model relative to the hook frame*/
};

}
#endif // ROBOTMODELCONFIG_HPP
