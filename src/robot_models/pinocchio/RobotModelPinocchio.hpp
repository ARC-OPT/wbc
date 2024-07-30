#ifndef ROBOT_MODEL_PINOCCHIO_HPP
#define ROBOT_MODEL_PINOCCHIO_HPP

#include "../../core/RobotModel.hpp"
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace wbc {

class RobotModelPinocchio : public RobotModel{
protected:
    static RobotModelRegistry<RobotModelPinocchio> reg;

    pinocchio::Model model;
    typedef std::shared_ptr<pinocchio::Data> DataPtr;
    DataPtr data;

    /** Free all data*/
    void clear();
public:
    RobotModelPinocchio();
    ~RobotModelPinocchio();

    /**
     * @brief This will read the robot model from the given URDF file and initialize all members accordingly.
     * @param cfg Model configuration. See RobotModelConfig.hpp for details
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg);

    /** @brief Update kinematics/dynamics.
      * @param joint_positions Positions of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_velocities Velocities of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_acceleration Velocities of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param fb_pose Pose of the floating base in world coordinates
      * @param fb_twist Twist of the floating base in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame attached to the floating base (robot root),
      * but aligned to world coordinates
      * @param fb_acc Spatial accelerationof the floating base in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame attached to the floating base (robot root),
      * but aligned to world coordinates
    */
    virtual void update(const Eigen::VectorXd& joint_positions,
                        const Eigen::VectorXd& joint_velocities,
                        const Eigen::VectorXd& joint_accelerations,
                        const types::Pose& fb_pose,
                        const types::Twist& fb_twist,
                        const types::SpatialAcceleration& fb_acc);

    /** Returns the pose of the body defined by frame_id. Pose will be computed with respect to world frame (floating base robot) or
     *  robot root link (fixed base)*/
    virtual const types::Pose &pose(const std::string &frame_id, const bool recompute = false);

    /** Returns the twist of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::Twist &twist(const std::string &frame_id, const bool recompute = false);

    /** Returns the spatial acceleration of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::SpatialAcceleration &acceleration(const std::string &frame_id, const bool recompute = false);

    /** @brief Returns the Space Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param frame_id Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN matrix, where N is the number of robot joints
      */
    virtual const Eigen::MatrixXd &spaceJacobian(const std::string &frame_id, const bool recompute = false);

    /** @brief Returns the Body Jacobian for the kinematic chain between root and the tip frame as full body Jacobian. Size of the Jacobian will be 6 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot. The columns that correspond to joints that are not part of the kinematic chain will have only zeros as entries.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param frame_id Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN matrix, where N is the number of robot joints
      */
    virtual const Eigen::MatrixXd &bodyJacobian(const std::string &frame_id, const bool recompute = false);

    /** @brief Returns the CoM Jacobian for the entire robot, which maps the robot joint velocities to linear spatial velocities in robot base coordinates.
      * Size of the Jacobian will be 3 x nJoints, where nJoints is the number of joints of the whole robot. The order of the
      * columns will be the same as the configured joint order of the robot.
      * @return A 3xN matrix, where N is the number of robot joints
      */
    virtual const Eigen::MatrixXd &comJacobian(const bool recompute = false);

    /** @brief Returns the spatial acceleration bias, i.e. the term Jdot*qdot
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param frame_id Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A Nx1 vector, where N is the number of robot joints
      */
    virtual const types::SpatialAcceleration &spatialAccelerationBias(const std::string &frame_id, const bool recompute = false);

    /** @brief Compute and return the joint space mass-inertia matrix, which is nj x nj, where nj is the number of joints of the system*/
    virtual const Eigen::MatrixXd &jointSpaceInertiaMatrix(const bool recompute = false);

    /** @brief Compute and return the bias force vector, which is nj x 1, where nj is the number of joints of the system*/
    virtual const Eigen::VectorXd &biasForces(const bool recompute = false);

    /** @brief Compute and return center of mass expressed in base frame*/
    virtual const types::RigidBodyState& centerOfMass(const bool recompute = false);

    /** @brief Compute tau from internal state
      * @param qdd_ref (Optional) Desired reference joint acceleration (without floating base), if empty actual acceleration will be used
      * @param f_ext (Optional) Contact wrenches, if not empty, size has to match the number of contact points. All wrenches will be in local coordinates, but aligned to world*/
    virtual const Eigen::VectorXd& inverseDynamics(const Eigen::VectorXd& qdd_ref = Eigen::VectorXd(),
                                                   const std::vector<types::Wrench>& f_ext = std::vector<types::Wrench>());

};

}

#endif
