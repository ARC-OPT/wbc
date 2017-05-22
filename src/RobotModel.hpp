#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <map>

namespace base{
    namespace samples{
        class Joints;
        class RigidBodyState;
    }
}

namespace wbc{

/** Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC*/
class RobotModel{

protected:
    std::vector<std::string> joint_names;
    std::string base_frame;

public:
    RobotModel(const std::vector<std::string> &joint_names, const std::string &base_frame) :
        joint_names(joint_names),
        base_frame(base_frame){}

    virtual ~RobotModel(){}

    /**
     * @brief Update the robot model
     * @param joint_state The joint_state vector. Has to contain all robot joints.
     * @param poses Optionally update links of the robot model. This can be used to update e.g. the relative position between two robots in the model.
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>()) = 0;

    /** Returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates!*/
    virtual const base::samples::RigidBodyState &rigidBodyState(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** Returns the current status of the given joint names */
    virtual const base::samples::Joints& jointState(const std::vector<std::string> &joint_names) = 0;

    /** Returns the Jacobian for the kinematic chain between root and the tip frame. By convention the Jacobian is computed with respect to
        the root frame with the rotation point at the tip frame*/
    virtual const base::MatrixXd &jacobian(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** Return the overall number of joints in the robot model*/
    uint noOfJoints(){return joint_names.size();}

    /** Return all joint names*/
    const std::vector<std::string>& jointNames(){return joint_names;}

    /** Get index of joint name*/
    uint jointIndex(const std::string &joint_name){
        uint idx = std::find(joint_names.begin(), joint_names.end(), joint_name) - joint_names.begin();
        if(idx >= joint_names.size())
            throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
        return idx;
    }

    /** Get the base frame of the robot*/
    const std::string& baseFrame(){return base_frame;}

};
}

#endif // ROBOTMODEL_HPP
