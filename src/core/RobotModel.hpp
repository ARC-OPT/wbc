#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <memory>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>
#include <base/Acceleration.hpp>
#include <base/JointLimits.hpp>
#include <base/samples/Wrenches.hpp>
#include <base/commands/Joints.hpp>
#include "RobotModelConfig.hpp"
#include <urdf_world/world.h>

namespace wbc{

std::vector<std::string> operator+(std::vector<std::string> a, std::vector<std::string> b);

/**
 * @brief Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC
 */
class RobotModel{
protected:
    void clear();

    /** ID of kinematic chain given root and tip*/
    const std::string chainID(const std::string& root, const std::string& tip){return root + "_" + tip;}

    ActiveContacts active_contacts;
    base::Vector3d gravity;
    base::samples::RigidBodyStateSE3 floating_base_state;
    base::samples::Wrenches contact_wrenches;
    RobotModelConfig robot_model_config;
    std::string world_frame, base_frame;
    base::JointLimits joint_limits;
    std::vector<std::string> actuated_joint_names;
    std::vector<std::string> independent_joint_names;
    std::vector<std::string> joint_names;
    std::vector<std::string> joint_names_floating_base;
    urdf::ModelInterfaceSharedPtr robot_urdf;
    bool has_floating_base;
    base::samples::Joints joint_state;
    base::MatrixXd joint_space_inertia_mat;
    base::MatrixXd com_jac;
    base::VectorXd bias_forces;
    base::Acceleration spatial_acc_bias;
    base::MatrixXd selection_matrix;
    base::samples::RigidBodyStateSE3 com_rbs;
    base::samples::RigidBodyStateSE3 rbs;

    typedef std::map<std::string, base::MatrixXd > JacobianMap;
    JacobianMap space_jac_map;
    JacobianMap body_jac_map;
    JacobianMap jac_dot_map;

    // Helper
    base::samples::Joints joint_state_out;

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
    const base::samples::Joints& jointState(const std::vector<std::string> &joint_names);

    /** Return entire system state*/
    virtual void systemState(base::VectorXd &q, base::VectorXd &qd, base::VectorXd &qdd) = 0;

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

    /** @brief Returns the CoM Jacobian for the entire robot, which maps the robot joint velocities to linear spatial velocities in robot base coordinates.
      * Size of the Jacobian will be 3 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot.
      * @return A 3xN matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &comJacobian() = 0;

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
    const std::vector<std::string>& jointNames(){return joint_names;}

    /** @brief Return only actuated joint names*/
    const std::vector<std::string>& actuatedJointNames(){return actuated_joint_names;}

    /** @brief Return only independent joint names*/
    const std::vector<std::string>& independentJointNames(){return independent_joint_names;}

    /** @brief Get index of joint name*/
    uint jointIndex(const std::string &joint_name);

    /** @brief Get the base frame of the robot*/
    const std::string& baseFrame(){return base_frame;}

    /** @brief Get the world frame id*/
    const std::string& worldFrame(){return world_frame;}

    /** @brief Return current joint limits*/
    const base::JointLimits& jointLimits(){return joint_limits;}

    /** @brief Return True if given link name is available in robot model, false otherwise*/
    bool hasLink(const std::string& link_name);

    /** @brief Return True if given joint name is available in robot model, false otherwise*/
    bool hasJoint(const std::string& joint_name);

    /** @brief Return True if given joint name is an actuated joint in robot model, false otherwise*/
    bool hasActuatedJoint(const std::string& joint_name);

    /** @brief Return current selection matrix S that maps complete joint vector to actuated joint vector: q_a = S * q. The matrix
      * consists of only zeros and ones. Size is na x nq, where na is the number of actuated joints and
      * nq the total number of joints. */
    const base::MatrixXd& selectionMatrix(){return selection_matrix;}

    /** @brief Compute and return center of mass expressed in base frame*/
    virtual const base::samples::RigidBodyStateSE3& centerOfMass() = 0;

    /** @brief Provide information about which link is currently in contact with the environment*/
    void setActiveContacts(const ActiveContacts &contacts);

    /** @brief Provide links names that are possibly in contact with the environment (typically the end effector links)*/
    const ActiveContacts& getActiveContacts(){return active_contacts;}

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
    virtual void computeInverseDynamics(base::commands::Joints &solver_output) = 0;

    /** @brief Get current robot model config*/
    const RobotModelConfig& getRobotModelConfig(){return robot_model_config;}

    /** @brief Is floating base robot?*/
    bool hasFloatingBase(){return has_floating_base;}

    /** @brief Load URDF model from either file or string*/
    urdf::ModelInterfaceSharedPtr loadRobotURDF(const std::string& file_or_string);

};
typedef std::shared_ptr<RobotModel> RobotModelPtr;

template<typename T> RobotModel* createT(){return new T;}

struct RobotModelFactory{
    typedef std::map<std::string, RobotModel*(*)()> RobotModelMap;

    static RobotModel *createInstance(const std::string& name) {
        RobotModelMap::iterator it = getRobotModelMap()->find(name);
        if(it == getRobotModelMap()->end())
            throw std::runtime_error("Failed to create instance of plugin " + name + ". Is the plugin registered?");
        return it->second();
    }

    template<typename T>
    static T* createInstance(const std::string& name){
        RobotModel* tmp = createInstance(name);
        T* ret = dynamic_cast<T*>(tmp);
        return ret;
    }

    static RobotModelMap *getRobotModelMap(){
        if(!robot_model_map)
            robot_model_map = new RobotModelMap;
        return robot_model_map;
    }

    static void clear(){
        robot_model_map->clear();
    }
private:
    static RobotModelMap *robot_model_map;
};

template<typename T>
struct RobotModelRegistry : RobotModelFactory{
    RobotModelRegistry(const std::string& name) {
        RobotModelMap::iterator it = getRobotModelMap()->find(name);
        if(it != getRobotModelMap()->end())
            throw std::runtime_error("Failed to register plugin with name " + name + ". A plugin with the same name is already registered");
        getRobotModelMap()->insert(std::make_pair(name, &createT<T>));
    }
};

}

#endif // ROBOTMODEL_HPP
