#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <memory>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>
#include <base/Acceleration.hpp>
#include <base/JointLimits.hpp>
#include <base/samples/Wrenches.hpp>

namespace wbc{

class RobotModelConfig;

std::vector<std::string> operator+(std::vector<std::string> a, std::vector<std::string> b);

/**
 * @brief Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC
 */
class RobotModel{
protected:
    void updateFloatingBase(const base::samples::RigidBodyStateSE3& rbs,
                            const std::vector<std::string> &floating_base_virtual_joint_names,
                            base::samples::Joints& joint_state);

    std::vector<std::string> contact_points;
    base::Vector3d gravity;
    base::samples::RigidBodyStateSE3 floating_base_state;
    base::samples::Wrenches contact_wrenches;
    base::VectorXd tau_computed;

public:
    RobotModel();
    virtual ~RobotModel(){}

    /**
     * @brief Load and configure the robot model
     * @param cfg Model configuration. See RobotModelConfig.hpp for details
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg) = 0;

    /**
     * @brief Update the robot configuration
     * @param joint_state The joint_state vector. Has to contain all robot joints that are configured in the model.
     * @param poses Optional, only for floating base robots: update the floating base state of the robot model.
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const base::samples::RigidBodyStateSE3& floating_base_state = base::samples::RigidBodyStateSE3()) = 0;

    /** Returns the current status of the given joint names */
    virtual const base::samples::Joints& jointState(const std::vector<std::string> &joint_names) = 0;

    /** Returns the pose, twist and spatial acceleration between the two given frames. All quantities are defined in root_frame coordinates*/
    virtual const base::samples::RigidBodyStateSE3 &rigidBodyState(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** @brief Returns the Space Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &spaceJacobian(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** @brief Returns the Body Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &bodyJacobian(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** @brief Returns the spatial acceleration bias, i.e. the term Jdot*qdot
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A Nx1 vector, where N is the number of robot joints
      */
    virtual const base::Acceleration &spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** @brief Returns the derivative of the Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. By convention reference frame & reference point
      *  of the Jacobian will be the root frame (corresponding to the body Jacobian). Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &jacobianDot(const std::string &root_frame, const std::string &tip_frame) = 0;

    /** @brief Compute and return the joint space mass-inertia matrix, which is nj x nj, where nj is the number of joints of the system*/
    virtual const base::MatrixXd &jointSpaceInertiaMatrix() = 0;

    /** @brief Compute and return the bias force vector, which is nj x 1, where nj is the number of joints of the system*/
    virtual const base::VectorXd &biasForces() = 0;

    /** @brief Return all joint names*/
    virtual const std::vector<std::string>& jointNames() = 0;

    /** @brief Return only actuated joint names*/
    virtual  const std::vector<std::string>& actuatedJointNames() = 0;

    /** @brief Get index of joint name*/
    virtual uint jointIndex(const std::string &joint_name) = 0;

    /** @brief Get the base frame of the robot*/
    virtual const std::string& baseFrame() = 0;

    /** @brief Return current joint limits*/
    virtual const base::JointLimits& jointLimits() = 0;

    /** @brief Return current selection matrix S that maps complete joint vector to actuated joint vector: q_a = S * q. The matrix
      * consists of only zeros and ones. Size is na x nq, where na is the number of actuated joints and
      * nq the total number of joints. */
    virtual const base::MatrixXd& selectionMatrix() = 0;

    /** @brief Return True if given link name is available in robot model, false otherwise*/
    virtual bool hasLink(const std::string& link_name) = 0;

    /** @brief Return True if given joint name is available in robot model, false otherwise*/
    virtual bool hasJoint(const std::string& joint_name) = 0;

    /** @brief Return True if given joint name is an actuated joint in robot model, false otherwise*/
    virtual bool hasActuatedJoint(const std::string& joint_name) = 0;

    /** @brief Return Current center of gravity in expressed base frame*/
    virtual const base::samples::RigidBodyStateSE3& getCOM() = 0;

    /** @brief Provide links names that are possibly in contact with the environment (typically the end effector links)*/
    void setContactPoints(const std::vector<std::string> contacts){contact_points=contacts;}

    /** @brief Provide links names that are possibly in contact with the environment (typically the end effector links)*/
    const std::vector<std::string>& getContactPoints(){return contact_points;}

    /** @brief Return number of joints*/
    uint noOfJoints(){return jointNames().size();}

    /** @brief Return number of actuated/active joints*/
    uint noOfActuatedJoints(){return actuatedJointNames().size();}

    /** @brief Set the current gravity vector*/
    void setGravityVector(const base::Vector3d& g){gravity=g;}

    /** @brief Get current status of floating base*/
    const base::samples::RigidBodyStateSE3& floatingBaseState(){return floating_base_state;}

    /** @brief Set contact wrenches, names have to consistent with the configured contact points*/
    void setContactWrenches(const base::samples::Wrenches& wrenches){contact_wrenches = wrenches;}

    /** @brief Compute and return the inverse dynamics solution*/
    virtual const base::VectorXd& computeInverseDynamics() = 0;

};
typedef std::shared_ptr<RobotModel> RobotModelPtr;

}

#endif // ROBOTMODEL_HPP
