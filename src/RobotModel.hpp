#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include <base/Eigen.hpp>

namespace base{
    namespace samples{
        class Joints;
        class RigidBodyState;
    }
}

namespace wbc{

/** Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC*/
class RobotModel{

public:
    RobotModel(){}
    virtual ~RobotModel(){}

    /**
     * @brief Update the robot model
     * @param joint_state The joint_state vector. Has to contain all robot joints.
     * @param poses Optionally update links of the robot model. This can be used to update e.g. the relative position between two robots in the model.
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>()) = 0;

    /** Returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates!*/
    virtual void rigidBodyState(const std::string &root_frame,
                                const std::string &tip_frame,
                                base::samples::RigidBodyState& rigid_body_state) = 0;

    /** Returns the current status of the given joint names */
    virtual void jointState(const std::vector<std::string> &joint_names,
                            base::samples::Joints& joint_state) = 0;

    /** Returns the Jacobian for the kinematic chain between root and the tip frame. By convention the Jacobian is computed with respect to
        the root frame with the rotation point at the tip frame*/
    virtual void jacobian(const std::string &root_frame,
                          const std::string &tip_frame,
                          base::MatrixXd &jacobian) = 0;
};
}

#endif // ROBOTMODEL_HPP
