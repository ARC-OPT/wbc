#ifndef WBC_CORE_ROBOTMODEL_HPP
#define WBC_CORE_ROBOTMODEL_HPP

#include <memory>
#include <map>
#include "../types/JointLimits.hpp"
#include "../types/RigidBodyState.hpp"
#include "../types/JointState.hpp"
#include "../types/Wrench.hpp"
#include "RobotModelConfig.hpp"
#include <urdf_world/world.h>

namespace wbc{

/**
 * @brief Interface for all robot models. This has to provide all kinematics and dynamics information that is required for WBC.
 * Conventions and usage:
 *   1. All quantities will be computed with respect to world coordinates (floating base robot) or robot root link (fixed base robot)
 *   2. Joint order is alphabetic, with the floating base being the first 6 joints:
 *       [floating_x,floating_y,floating_z,floating_rx,floating_rx,floating_rz,
 *        q1 ... qn]
 *     Call jointNames() to get the order of joints (without floating base)
 *   3. You have to call configure() once and update() in every cycle (to provide new position,velocity,accelerations
 *     readings) before you request any kinematics or dynamics data
 */
class RobotModel{
protected:
    void clear();

    std::vector<Contact> contacts;
    Eigen::Vector3d gravity;
    types::RigidBodyState floating_base_state;
    RobotModelConfig robot_model_config;
    std::string world_frame, base_frame;
    types::JointLimits joint_limits;
    std::vector<std::string> actuated_joint_names;
    std::vector<std::string> independent_joint_names;
    std::vector<std::string> joint_names;
    urdf::ModelInterfaceSharedPtr robot_urdf;
    bool has_floating_base;
    types::JointState joint_state;
    Eigen::MatrixXd joint_space_inertia_mat;
    Eigen::MatrixXd com_jac;
    Eigen::VectorXd bias_forces;
    types::SpatialAcceleration zero_acc;
    Eigen::MatrixXd selection_matrix;
    types::RigidBodyState com_rbs;
    Eigen::VectorXd q, qd, qdd, tau, zero_jnt;
    bool configured,updated;
    std::map<std::string,Eigen::MatrixXd> space_jac_map;
    std::map<std::string,Eigen::MatrixXd> body_jac_map;
    std::map<std::string,types::Pose> pose_map;
    std::map<std::string,types::Twist> twist_map;
    std::map<std::string,types::SpatialAcceleration> acc_map;
    std::map<std::string,types::SpatialAcceleration> spatial_acc_bias_map;
    bool compute_inertia_mat;
    bool compute_com;
    bool compute_com_jac;
    bool compute_bias_forces;

public:
    RobotModel();
    virtual ~RobotModel(){}

    /**
     * @brief This will read the robot model from the given URDF file and initialize all members accordingly.
     * @param cfg Model configuration. See RobotModelConfig.hpp for details
     * @return True in case of success, else false
     */
    virtual bool configure(const RobotModelConfig& cfg) = 0;

    /** @brief Update kinematics/dynamics for fixed base robots. Joint acceleration will be assumed zero.
      * @param joint_positions Positions of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_velocities Velocities of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      */
    void update(const Eigen::VectorXd& joint_positions,
                const Eigen::VectorXd& joint_velocities);

    /** @brief Update kinematics/dynamics for fixed base robots.
      * @param joint_positions Positions of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_velocities Velocities of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_accelerations Accelerations of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      */
    virtual void update(const Eigen::VectorXd& joint_positions,
                const Eigen::VectorXd& joint_velocities,
                const Eigen::VectorXd& joint_accelerations);

    /** @brief Update kinematics/dynamics for floating base robots. Joint and spatial acceleration will be assumed zero.
      * @param joint_positions Positions of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param joint_velocities Velocities of all independent joints. These have to be in the same order as used by the model (alphabetic). For getting the joint order, call jointNames().
      * @param fb_pose Pose of the floating base in world coordinates
      * @param fb_twist Twist of the floating base in "local-world-aligned" (hybrid) representation, i.e., the frame is attached to the floating base (robot root),
      * but aligned to world coordinates
      */
    void update(const Eigen::VectorXd& joint_positions,
                const Eigen::VectorXd& joint_velocities,
                const types::Pose& fb_pose,
                const types::Twist& fb_twist);

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
                        const types::SpatialAcceleration& fb_acc) = 0;

    /** Returns the current joint state (indepenent joints)*/
    const types::JointState& jointState(){return joint_state;}

    /** Returns the pose of the body defined by frame_id. Pose will be computed with respect to world frame (floating base robot) or
     *  robot root link (fixed base)*/
    virtual const types::Pose &pose(const std::string &frame_id, const bool recompute = false) = 0;

    /** Returns the twist of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::Twist &twist(const std::string &frame_id, const bool recompute = false) = 0;

    /** Returns the spatial acceleration of the body defined by frame_id. Twist will be in "local-world-aligned" (hybrid) representation, i.e., with respect to a frame
      * attached to the given body, but aligned to world coordinates (floating base robot) or robot root link (fixed base)*/
    virtual const types::SpatialAcceleration &acceleration(const std::string &frame_id, const bool recompute = false) = 0;


    /** @brief Returns the Space Jacobian for the given frame. The order of the Jacobian's columns will be the same as in the model (alphabetic). Note that
      * the linear part of the jacobian will be aligned to world frame (floating base robot) or robot root link (fixed base), and the angular part will be in
      * body coordinates. This is refered to as "hybrid" representation.
      * @param frame_id Tip frame of the Jacobian. Has to be a valid link in the robot model.
      * @return A 6 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &spaceJacobian(const std::string &frame_id, const bool recompute = false) = 0;

    /** @brief Returns the Body Jacobian for the given frame. The order of the Jacobian's columns will be the same as in the model (alphabetic).
      * @param frame_id Reference frame of the Jacobian. Has to be a valid link in the robot model.
      * @return A 6 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &bodyJacobian(const std::string &frame_id, const bool recompute = false) = 0;

    /** @brief Returns the CoM Jacobian for the robot, which maps the robot joint velocities to linear spatial velocities of the CoM expressed in
      * world frame (floating base robot) or robot root link (fixed base). The order of the columns will be the same as the configured joint order of the robot.
      * @return A 3 x nj matrix, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.
      */
    virtual const Eigen::MatrixXd &comJacobian(const bool recompute = false) = 0;

    /** @brief Returns the spatial acceleration bias, i.e. the term Jdot*qdot. The linear part of the acceleration will be aligned to world frame (floating base robot) or
      *  robot root link (fixed base), and the angular part will be in body coordinates. This is refered to as "hybrid" representation.
      * @param frame_id Reference frame of the spatial acceleration. Has to be a valid link in the robot model.
      * @return A 6 dof spatial acceleration.
      */
    virtual const types::SpatialAcceleration &spatialAccelerationBias(const std::string &frame_id, const bool recompute = false) = 0;

    /** @brief Returns the joint space mass-inertia matrix, which is nj x nj,  where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.*/
    virtual const Eigen::MatrixXd &jointSpaceInertiaMatrix(const bool recompute = false) = 0;

    /** @brief Returns the bias force vector, which is nj x 1, where nj is the number of joints + number of floating coordinates, i.e. 6 in case of a floating base robot.*/
    virtual const Eigen::VectorXd &biasForces(const bool recompute = false) = 0;

    /** @brief Return all joint names excluding the floating base. This will be
     * - The entire spanning tree if there are parallel loops
     * - Same as independent joints in case of a serial robot
     * - Same as actuated joint names in case of a fully actuated robot*/
    const std::vector<std::string>& jointNames(){return joint_names;}

    /** @brief Return only actuated joint names*/
    const std::vector<std::string>& actuatedJointNames(){return actuated_joint_names;}

    /** @brief Return only independent joint names.*/
    const std::vector<std::string>& independentJointNames(){return independent_joint_names;}

    /** @brief Get the internal index for a joint name*/
    uint jointIndex(const std::string &joint_name);

    /** @brief Get the base frame of the robot, i.e. the root of the URDF model*/
    const std::string& baseFrame(){return base_frame;}

    /** @brief Get the world frame id. Will be "world" in case of floating base robot and equal to base_frame in case of fixed base robot*/
    const std::string& worldFrame(){return world_frame;}

    /** @brief Return current joint limits*/
    const types::JointLimits jointLimits(){return joint_limits;}

    /** @brief Return current selection matrix, mapping actuated to all dof*/
    const Eigen::MatrixXd &selectionMatrix(){return selection_matrix;}

    /** @brief Return True if given link name is available in robot model, false otherwise*/
    bool hasLink(const std::string& link_name);

    /** @brief Return True if given joint name is available in robot model, false otherwise*/
    bool hasJoint(const std::string& joint_name);

    /** @brief Return True if given joint name is an actuated joint in robot model, false otherwise*/
    bool hasActuatedJoint(const std::string& joint_name);

    /** @brief Return centers of mass expressed in world frame*/
    virtual const types::RigidBodyState& centerOfMass(const bool recompute = false) = 0;

    /** @brief Set new contact points.*/
    void setContacts(const std::vector<Contact> &contacts);

    /** @brief get current contact points*/
    const std::vector<Contact>& getContacts(){return contacts;}

    /** @brief Return number of joints*/
    uint nj(){return jointNames().size() + (has_floating_base ? 6 : 0);}

    /** @brief Return number of actuated joints*/
    uint na(){return actuatedJointNames().size();}

    /** @brief Return dof of the floating base, will be either 0 (floating_base == false) or 6 (floating_base = = false) */
    uint nfb(){return has_floating_base ? 6 : 0;}

    /** Return number of active contacts*/
    uint nac();

    /** @brief Return total number of contact points*/
    uint nc(){return contacts.size();}

    /** @brief Return full system position vector, incl. floating base.
     *  Convention [floating_base_pos, floating_base_ori, joint_positions]:
     *    [fb_x,fb_y,fb_z,
     *     fb_qx,fb_qy,fb_qz,fb_qw,
     *     q_1 ... q_n)]*/
    const Eigen::VectorXd &getQ(){return q;}

    /** @brief Return full system velocity vector, incl. floating base.
     *  Convention [floating_base_vel, floating_base_rot_vel, joint_velocities]:
     *    [fb_vx,fb_vx,fb_vx,
     *     fb_wx,fb_wy,fb_wz,
     *     qd_1 ... qd_n)]*/
    const Eigen::VectorXd &getQd(){return qd;}

    /** @brief Return full system acceleration vector, incl. floating base.
     *  Convention [floating_base_acc, floating_base_rot_acc, joint_accelerations]:
     *    [fb_dvx,fb_dvx,fb_dvx,
     *     fb_dwx,fb_dwy,fb_dwz,
     *     qdd_1 ... qdd_n)]*/
    const Eigen::VectorXd &getQdd(){return qdd;}

    /** @brief Set the current gravity vector. Default is (0,0,-9.81)*/
    void setGravityVector(const Eigen::Vector3d& g){gravity=g;}

    /** @brief Get current status of floating base*/
    const types::RigidBodyState& floatingBaseState(){return floating_base_state;}

    /** @brief Get current robot model config*/
    const RobotModelConfig& getRobotModelConfig(){return robot_model_config;}

    /** @brief Is is a floating base robot?*/
    bool hasFloatingBase(){return has_floating_base;}

    /** @brief Load URDF model from either file or string*/
    urdf::ModelInterfaceSharedPtr loadRobotURDF(const std::string& file_or_string);

    /** @brief Return current URDF model*/
    urdf::ModelInterfaceSharedPtr getURDFModel(){return robot_urdf;}

    /** @brief Compute tau from internal state
      * @param qdd_ref (Optional) Desired reference joint acceleration (including floating base), if empty actual acceleration will be used.
      * @param f_ext (Optional) Contact wrenches, if not empty, size has to match the number of contact points. All wrenches will be in local coordinates, but aligned wrt. world*/
    virtual const Eigen::VectorXd& inverseDynamics(const Eigen::VectorXd& qdd_ref = Eigen::VectorXd(),
                                                   const std::vector<types::Wrench>& f_ext = std::vector<types::Wrench>()) = 0;

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

#endif // WBC_CORE_ROBOTMODEL_HPP
