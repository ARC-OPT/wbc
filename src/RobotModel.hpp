#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include <string>
#include <base/Eigen.hpp>

namespace base{
    namespace samples{
        class Joints;
        class RigidBodyState;
    }
}

namespace wbc{


/**
 * @brief Interface for all robot models.
 */
class RobotModel{

public:
    RobotModel(){}
    virtual ~RobotModel(){}

    /**
     * @brief Update the robot model with the current
     * @param joint_state The joint_state vector
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>()) = 0;


    virtual Eigen::Affine3d pose(const std::string &target_frame,
                                 const std::string &source_frame) = 0;

    virtual base::samples::Joints jointState(const std::string &target_frame,
                                             const std::string &source_frame) = 0;

    virtual Eigen::Matrix<double,6,Eigen::Dynamic> jacobian(const std::string &target_frame,
                                                            const std::string &source_frame) = 0;
};
}

#endif // ROBOTMODEL_HPP
