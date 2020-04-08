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

    std::string file;          /** Path to robot model file*/
    std::string hook;         /** Frame to which this robot model is attached in the overall model*/
    base::Pose initial_pose;  /** Initial pose of this model relative to the hook frame*/
};

}
#endif // ROBOTMODELCONFIG_HPP
