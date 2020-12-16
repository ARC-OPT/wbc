#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <base/Eigen.hpp>
#include <map>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <memory>
#include "RobotModelConfig.hpp"
#include <base/JointLimits.hpp>
#include <urdf_world/types.h>

namespace wbc{

/**
 * @brief Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC
 */
class RobotModel{
protected:
    std::vector<std::string> actuated_joint_names;
    std::string base_frame;
    base::JointLimits joint_limits;
    base::samples::Joints current_joint_state;
    base::Vector3d gravity;
    base::MatrixXd joint_space_inertia_mat;
    base::VectorXd bias_forces;
    base::Acceleration spatial_acc_bias;
    base::MatrixXd selection_matrix;
    base::samples::Joints joint_state_out;
    std::vector<std::string> floating_base_names;
    base::samples::RigidBodyStateSE3 floating_base_state;
    bool has_floating_base;
    std::vector<std::string> contact_points;
    std::vector<std::string> active_contacts;
    urdf::ModelInterfaceSharedPtr robot_urdf;

    void clear();
    void updateFloatingBase(const base::RigidBodyStateSE3& rbs, base::samples::Joints& joint_state);

public:
    RobotModel();
    virtual ~RobotModel();

    /**
     * @brief Load and configure the robot model
     * @param cfg Model configuration. See RobotModelConfig.hpp for details
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg) = 0;

    /**
     * @brief Update the robot configuration
     * @param joint_state The joint_state vector. Has to contain at least all robot joints that are configured in the model
     * @param poses Optional: update the floating base state of the robot model. Only for floating base robot
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const base::samples::RigidBodyStateSE3& floating_base_state = base::samples::RigidBodyStateSE3());

    /** Returns the current status of the given joint names */
    const base::samples::Joints& jointState(const std::vector<std::string> &joint_names);

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

    /** @brief Return the number of actuated joints in the robot model*/
    uint noOfActuatedJoints(){return actuated_joint_names.size();}

    /** @brief Return the overall number of joints in the robot model*/
    uint noOfJoints(){return current_joint_state.size();}

    /** @brief Return all joint names*/
    const std::vector<std::string>& jointNames(){return current_joint_state.names;}

    /** @brief Return only actuated joint names*/
    const std::vector<std::string>& actuatedJointNames(){return actuated_joint_names;}

    /** @brief Get index of joint name*/
    uint jointIndex(const std::string &joint_name);

    /** @brief Get the base frame of the robot*/
    const std::string& baseFrame(){return base_frame;}

    /** @brief Return current joint limits*/
    const base::JointLimits& jointLimits(){return joint_limits;}

    /** @brief Set the current gravity vector*/
    void setGravityVector(const base::Vector3d& g){gravity = g;}

    /** @brief Return the current gravity vector*/
    const base::Vector3d& getGravityVector(){return gravity;}

    /** @brief Return current selection matrix S that maps complete joint vector to actuated joint vector: q_a = S * q. The matrix
      * consists of only zeros and ones. Size is na x nq, where na is the number of actuated joints and
      * nq the total number of joints. */
    const base::MatrixXd& selectionMatrix(){return selection_matrix;}

    /** @brief Returns the current state of the floating base*/
    const base::samples::RigidBodyStateSE3& floatingBaseState(){return floating_base_state;}

    /** @brief Provide link names of active contacts*/
    void setActiveContacts(const std::vector<std::string> contacts){active_contacts = contacts;}

    /** @brief Provide links names that are possibly in contact with the environment (typically the end effector links)*/
    void setContactPoints(const std::vector<std::string> contacts){contact_points = contacts;}

    /** @brief Provide link names of active contacts*/
    const std::vector<std::string>& getActiveContacts(){return active_contacts;}

    /** @brief Provide links names that are possibly in contact with the environment (typically the end effector links)*/
    const std::vector<std::string>& getContactPoints(){return contact_points;}

    /** @brief Return True if given link name is available in robot model, false otherwise*/
    bool hasLink(const std::string& link_name);

    /** @brief Return True if given joint name is available in robot model, false otherwise*/
    bool hasJoint(const std::string& joint_name);
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;

}

#endif // ROBOTMODEL_HPP
