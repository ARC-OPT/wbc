#ifndef ROBOTMODELHYRODYN_HPP
#define ROBOTMODELHYRODYN_HPP

#include "../../core/RobotModel.hpp"

#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <wbc/types/JointCommand.hpp>

namespace wbc{

class RobotModelHyrodyn : public RobotModel{
private:
    static RobotModelRegistry<RobotModelHyrodyn> reg;

protected:
    hyrodyn::RobotModel_HyRoDyn hyrodyn;

    void clear();
    void addFloatingBaseToURDF(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::string &world_frame_id = "world");
public:
    RobotModelHyrodyn();
    virtual ~RobotModelHyrodyn();

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
    virtual const types::Pose &pose(const std::string &frame_id);

    /** Returns the twist of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::Twist &twist(const std::string &frame_id);

    /** Returns the spatial acceleration of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::SpatialAcceleration &acceleration(const std::string &frame_id);

    /** @brief Returns the Space Jacobian for the given frame. The order of the Jacobian's columns will be the same as in the model (alphabetic). Note that
      * the linear part of the jacobian will be aligned to world frame (floating base robot) or robot root link (fixed base), and the angular part will be in
      * body coordinates. This is refered to as "hybrid" representation.
      * @param frame_id Tip frame of the Jacobian. Has to be a valid link in the robot model.
      * @return A 6 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &spaceJacobian(const std::string &frame_id);

    /** @brief Returns the Body Jacobian for the given frame. The order of the Jacobian's columns will be the same as in the model (alphabetic).
      * @param frame_id Reference frame of the Jacobian. Has to be a valid link in the robot model.
      * @return A 6 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &bodyJacobian(const std::string &frame_id);

    /** @brief Returns the CoM Jacobian for the robot, which maps the robot joint velocities to linear spatial velocities of the CoM expressed in
      * world frame (floating base robot) or robot root link (fixed base). The order of the columns will be the same as the configured joint order of the robot.
      * @return A 3 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &comJacobian();

    /** @brief Returns the spatial acceleration bias, i.e. the term Jdot*qdot. The linear part of the acceleration will be aligned to world frame (floating base robot) or
      *  robot root link (fixed base), and the angular part will be in body coordinates. This is refered to as "hybrid" representation.
      * @param frame_id Reference frame of the spatial acceleration. Has to be a valid link in the robot model.
      * @return A 6 dof spatial acceleration.
      */
    virtual const types::SpatialAcceleration &spatialAccelerationBias(const std::string &frame_id);

    /** @brief Returns the joint space mass-inertia matrix, which is nj x nj,  where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.*/
    virtual const Eigen::MatrixXd &jointSpaceInertiaMatrix();

    /** @brief Returns the bias force vector, which is nj x 1, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.*/
    virtual const Eigen::VectorXd &biasForces();

    /** @brief Return centers of mass expressed in world frame*/
    virtual const types::RigidBodyState& centerOfMass();

    /** @brief Return pointer to the internal hyrodyn model*/
    hyrodyn::RobotModel_HyRoDyn *hyrodynHandle(){return &hyrodyn;}

    /** @brief Compute tau from internal state
      * @param qdd_ref (Optional) Desired reference joint acceleration (including floating base), if empty actual acceleration will be used.
      * @param f_ext (Optional) Contact wrenches, if not empty, size has to match the number of contact points. All wrenches will be in local coordinates, but aligned wrt. world*/
    virtual const Eigen::VectorXd& inverseDynamics(const Eigen::VectorXd& qdd_ref = Eigen::VectorXd(),
                                                   const std::vector<types::Wrench>& f_ext = std::vector<types::Wrench>());
};

}

#endif
