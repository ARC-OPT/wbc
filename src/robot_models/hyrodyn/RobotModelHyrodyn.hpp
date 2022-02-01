#ifndef ROBOTMODELHYRODYN_HPP
#define ROBOTMODELHYRODYN_HPP

#include "../../core/RobotModel.hpp"
#include "../../core/RobotModelConfig.hpp"

#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <urdf_world/types.h>
#include <base/commands/Joints.hpp>

namespace wbc{

class RobotModelHyrodyn : public RobotModel{
protected:
    base::samples::RigidBodyStateSE3 rbs;
    std::string base_frame;
    base::JointLimits joint_limits;
    base::samples::Joints joint_state;
    base::MatrixXd joint_space_inertia_mat;
    base::VectorXd bias_forces;
    base::Acceleration spatial_acc_bias;
    base::MatrixXd selection_matrix;
    base::samples::Joints joint_state_out;
    std::vector<std::string> joint_names;
    std::vector<std::string> independent_joint_names;
    std::vector<std::string> joint_names_floating_base;
    base::samples::RigidBodyStateSE3 floating_base_state;
    urdf::ModelInterfaceSharedPtr robot_urdf;
    base::samples::RigidBodyStateSE3 com_rbs;
    base::MatrixXd jacobian;
    hyrodyn::RobotModel_HyRoDyn hyrodyn;

    void clear();
public:
    RobotModelHyrodyn();
    virtual ~RobotModelHyrodyn();

    /**
     * @brief Load and configure the robot model. In this implementation, each model config constains a URDF file that will be parsed into a KDL tree.
     *  If the overall model is empty, the overall KDL::Tree will be replaced by the given tree. If there
     *  is already a KDL Tree, the new tree will be attached with the given pose to the 'hook' frame of the overall tree. The relative poses
     *  of the trees can be updated online by calling update() with poses parameter appropriately set.
     * @param model_config The models configuration(s). These include the path to the URDF model file(s), the relative poses and hooks
     *  to which the models shall be attached. This way you can add multiple robot model tree and attach them to each other.
     * @param base_frame Base frame of the model. If left empty, the base will be selected as the root frame of the first URDF model.
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg);

    /**
     * @brief Update the robot model. The joint state has to contain all joints that are relevant in the model. This means: All joints that are ever required
     *  when requesting rigid body states, Jacobians or joint states. Note that
     * @param joint_state The joint_state vector. Has to contain all robot joints.
     * @param poses Optionally update links of the robot model. This can be used to update e.g. the relative position between two robots in the model. The source frame
     *  of the given rigid body state has to match the segment name in the KDL Tree that shall be updated
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const base::samples::RigidBodyStateSE3& floating_base_state = base::samples::RigidBodyStateSE3());

    /** Returns the current status of the given joint names */
    virtual const base::samples::Joints& jointState(const std::vector<std::string> &joint_names);

    /**
     * @brief Computes and returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates.
     *  This will create a kinematic chain between root and tip frame, if called for the first time with the given arguments.
     * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
     * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
     */
    virtual const base::samples::RigidBodyStateSE3 &rigidBodyState(const std::string &root_frame, const std::string &tip_frame);

    /** @brief Returns the Space Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      */
    virtual const base::MatrixXd &spaceJacobian(const std::string &root_frame, const std::string &tip_frame);

    /** @brief Returns the Body Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      */
    virtual const base::MatrixXd &bodyJacobian(const std::string &root_frame, const std::string &tip_frame);

    /** @brief Returns the derivative of the Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. By convention reference frame & reference point
      *  of the Jacobian will be the root frame (corresponding to the body Jacobian). Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN Jacobian derivative matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &jacobianDot(const std::string &root_frame, const std::string &tip_frame);

    /** @brief Returns the spatial acceleration bias, i.e. the term Jdot*qdot
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A Nx1 vector, where N is the number of robot joints
      */
    virtual const base::Acceleration &spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame);

    /** Compute and return the joint space mass-inertia matrix, which is nj x nj, where nj is the number of joints of the system*/
    virtual const base::MatrixXd &jointSpaceInertiaMatrix();

    /** Compute and return the bias force vector, which is nj x 1, where nj is the number of joints of the system*/
    virtual const base::VectorXd &biasForces();

    /** @brief Return all joint names*/
    virtual const std::vector<std::string>& jointNames(){return joint_names;}

    /** @brief Return only actuated joint names*/
   virtual  const std::vector<std::string>& actuatedJointNames(){return hyrodyn.jointnames_active;}

    /** @brief Return only independent joint names*/
    virtual  const std::vector<std::string>& independentJointNames(){return independent_joint_names;}

    /** @brief Get index of joint name*/
    virtual uint jointIndex(const std::string &joint_name);

    /** @brief Get the base frame of the robot*/
    virtual const std::string& baseFrame(){return base_frame;}

    /** @brief Return current joint limits*/
    virtual const base::JointLimits& jointLimits(){return joint_limits;}

    /** @brief Return current selection matrix S that maps complete joint vector to actuated joint vector: q_a = S * q. The matrix
      * consists of only zeros and ones. Size is na x nq, where na is the number of actuated joints and
      * nq the total number of joints. */
    virtual const base::MatrixXd& selectionMatrix(){return selection_matrix;}

    /** @brief Return True if given link name is available in robot model, false otherwise*/
    virtual bool hasLink(const std::string& link_name);

    /** @brief Return True if given joint name is available in robot model, false otherwise*/
    virtual bool hasJoint(const std::string& joint_name);

    /** @brief Return True if given joint name is an actuated joint in robot model, false otherwise*/
    virtual bool hasActuatedJoint(const std::string& joint_name);

    /** @brief Return Current center of gravity in expressed base frame*/
    virtual const base::samples::RigidBodyStateSE3& centerOfMass();

    /** @brief Return pointer to the internal hyrodyn model*/
    hyrodyn::RobotModel_HyRoDyn *hyrodynHandle(){return &hyrodyn;}

    /** @brief Compute and return the inverse dynamics solution*/
    virtual void computeInverseDynamics(base::commands::Joints &solver_output);
};
}

#endif
