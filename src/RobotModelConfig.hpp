#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/Pose.hpp>
#include <base/NamedVector.hpp>
#include <wbc_common/CartesianState.hpp>

namespace wbc{

/**
 * @brief Robot Model configuration class
 */
class RobotModelConfig{
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
    ~RobotModelConfig(){
    }

    std::string file;          /** Path to robot model file*/
    std::string hook;         /** Frame to which this robot model is attached in the overall model*/
    base::Pose initial_pose;  /** Initial pose of this model relative to the hook frame*/
};

class RobotModelsState : public base::NamedVector<wbc::CartesianState>{
};

}
#endif // ROBOTMODELCONFIG_HPP
