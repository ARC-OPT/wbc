#ifndef ROBOTMODELHYRODYN_HPP
#define ROBOTMODELHYRODYN_HPP

#include "../../core/RobotModel.hpp"

#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <base/commands/Joints.hpp>

namespace wbc{

class RobotModelHyrodyn : public RobotModel{
private:
    static RobotModelRegistry<RobotModelHyrodyn> reg;

protected:
    hyrodyn::RobotModel_HyRoDyn hyrodyn;

    void clear();
public:
    RobotModelHyrodyn();
    virtual ~RobotModelHyrodyn();

    /**
     * @brief Load and configure the robot model
     * @param cfg Model configuration. See RobotModelConfig.hpp for details
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg);

    /**
     * @brief Update the robot configuration
     * @param joint_state The joint_state vector. Has to contain all robot joints that are configured in the model.
     * @param poses Optional, only for floating base robots: update the floating base state of the robot model.
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const base::samples::RigidBodyStateSE3& floating_base_state = base::samples::RigidBodyStateSE3());

    /** Return entire system state*/
    virtual void systemState(base::VectorXd &q, base::VectorXd &qd, base::VectorXd &qdd);

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

    /** @brief Returns the CoM Jacobian for the entire robot, which maps the robot joint velocities to linear spatial velocities in robot base coordinates.
      * Size of the Jacobian will be 3 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot.
      * @return A 3xN matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &comJacobian();

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

    /** @brief Return Current center of gravity in expressed base frame*/
    virtual const base::samples::RigidBodyStateSE3& centerOfMass();

    /** @brief Return pointer to the internal hyrodyn model*/
    hyrodyn::RobotModel_HyRoDyn *hyrodynHandle(){return &hyrodyn;}

    /** @brief Compute and return the inverse dynamics solution*/
    virtual void computeInverseDynamics(base::commands::Joints &solver_output);
};

}

#endif
