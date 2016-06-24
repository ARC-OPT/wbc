#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class RobotModelConfig{
public:
    RobotModelConfig(){
        initial_pose.setTransform(Eigen::Affine3d::Identity());
    }
    RobotModelConfig(const std::string& _file,
                     const base::samples::RigidBodyState& _initial_pose = base::samples::RigidBodyState(),
                     const std::string& _hook = "") :
        file(_file),
        hook(_hook),
        initial_pose(_initial_pose){
    }

    std::string file;
    std::string hook;
    base::samples::RigidBodyState initial_pose;
};
}
#endif // ROBOTMODELCONFIG_HPP
