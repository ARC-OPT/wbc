#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include "../core/RobotModel.hpp"

#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <base/samples/Joints.hpp>
#include <memory>

namespace wbc{

class KinematicChainKDL;

/**
 *  @brief This model describes the kinemetic relationships required for velocity based wbc. It is based on a single KDL Tree. However, multiple KDL trees can be added
 *  and will be appropriately concatenated. This way you can describe e.g. geometric robot-object relationships or create multi-robot scenarios.
 */
class KinematicRobotModelKDL : public RobotModel{

    typedef std::shared_ptr<KinematicChainKDL> KinematicChainKDLPtr;
    typedef std::map<std::string, KinematicChainKDLPtr> KinematicChainKDLMap;
    typedef std::map<std::string, base::MatrixXd > JacobianMap;
    const std::string virtual_joint_names[6] = {"_x", "_y", "_z", "_rot_x", "_rot_y", "_rot_z"};

protected:
    KDL::Tree full_tree;                        /** Overall kinematic tree*/
    base::samples::Joints current_joint_state;  /** Last joint state that was passed through the call of update()*/
    base::samples::Joints virtual_joint_state;  /** Last virtual joint state */
    KinematicChainKDLMap kdl_chain_map;         /** Map of KDL Chains*/
    JacobianMap jac_map;                        /** Map of robot jacobians for all kinematic chains*/
    JacobianMap jac_dot_map;                    /** Map of robot jacobian derivatives for all kinematic chains*/
    base::samples::Joints joint_state_out;      /** Helper variable*/

    /**
     * @brief Create a KDL chain and add it to the KDL Chain map. Throws an exception if chain cannot be extracted from KDL Tree
     * @param root_frame Root frame of the chain
     * @param tip_frame Tip frame of the chain
     */
    void createChain(const std::string &root_frame, const std::string &tip_frame);

    /** Add a KDL Tree to the model. If the model is empty, the overall KDL::Tree will be replaced by the given tree. If there
     *  is already a KDL Tree, the new tree will be attached with the given pose to the hook frame of the overall tree. The relative poses
     *  of the trees can be updated online by calling update() with poses parameter appropriately set. This will also create the
     *  joint index map*/

    bool addTree(const KDL::Tree& tree, const std::string& hook = "", const base::Pose &pose = base::Pose());
    /** Free storage and clear data structures*/
    void clear();

    /** ID of kinematic chain given root and tip*/
    const std::string chainID(const std::string& root, const std::string& tip){return root + "_" + tip;}

    bool addVirtual6DoFJoint(const std::string &hook, const std::string& tip, const base::Pose& initial_pose);

    /** Update the position and orientation of a tree that is attached to the initial robot (see configure for details). */
    void updateVirtual6DoFJoint(const base::RigidBodyStateSE3& state, const std::string &tip_frame);

public:
    KinematicRobotModelKDL();
    virtual ~KinematicRobotModelKDL();

    /**
     * @brief Load and configure the robot model with single model file
     * @param model_file The models configuration file.
     * @param joint_names Order of joint names within the model.
     * @param base_frame Base frame of the model.
     * @return True in case of success, else false
     */
    virtual bool configure(const std::string& model_filename,
                           const std::vector<std::string> &joint_names = std::vector<std::string>(),
                           const std::string &base_frame = "");

    /**
     * @brief Load and configure the robot model. In this implementation, each model config constains a URDF file that will be parsed into a KDL tree.
     *  If the overall model is empty, the overall KDL::Tree will be replaced by the given tree. If there
     *  is already a KDL Tree, the new tree will be attached with the given pose to the 'hook' frame of the overall tree. The relative poses
     *  of the trees can be updated online by calling update() with poses parameter appropriately set.
     * @param model_config The models configuration(s). These include the path to the URDF model file(s), the relative poses and hooks
     *  to which the models shall be attached. This way you can add multiple robot model tree and attach them to each other.
     * @param joint_names Order of joint names within the model. If left empty, the order will be the same as in the KDL tree (alphabetical)
     * @param base_frame Base frame of the model. If left empty, the base will be selected as the root frame of the first URDF model.
     * @return True in case of success, else false
     */
    virtual bool configure(const std::vector<RobotModelConfig>& model_config,
                           const std::vector<std::string> &joint_names = std::vector<std::string>(),
                           const std::string &base_frame = "");

    /**
     * @brief Update the robot model. The joint state has to contain all joints that are relevant in the model. This means: All joints that are ever required
     *  when requesting rigid body states, Jacobians or joint states. Note that
     * @param joint_state The joint_state vector. Has to contain all robot joints.
     * @param poses Optionally update links of the robot model. This can be used to update e.g. the relative position between two robots in the model. The source frame
     *  of the given rigid body state has to match the segment name in the KDL Tree that shall be updated
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const base::samples::RigidBodyStatesSE3& virtual_joint_states = base::samples::RigidBodyStatesSE3());

    /**
     * @brief Computes and returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates.
     *  This will create a kinematic chain between root and tip frame, if called for the first time with the given arguments.
     * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
     * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
     */
    virtual const base::samples::RigidBodyStateSE3 &rigidBodyState(const std::string &root_frame, const std::string &tip_frame);

    /**
     * @brief Returns the current status of the given joint names
     * @param joint_names Has to contain only valid joints from the robot model
     */
    virtual const base::samples::Joints& jointState(const std::vector<std::string> &joint_names);

    /** @brief Returns the Jacobian for the kinematic chain between root and the tip frame. By convention reference frame & reference point
      *  of the Jacobian will be the root frame.
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      */
    virtual const base::MatrixXd &jacobian(const std::string &root_frame, const std::string &tip_frame);

    /** @brief Returns the derivative of the Jacobian for the kinematic chain between root and the tip frame. By convention the Jacobian is computed with respect to
      *        the root frame with the rotation point at the tip frame
      * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
      * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
      * @return A 6xN Jacobian derivative matrix, where N is the number of robot joints
      */
    virtual const base::MatrixXd &jacobianDot(const std::string &root_frame, const std::string &tip_frame);

    /** Check if a frame is available in the model*/
    bool hasFrame(const std::string &name);

    /** Return all non-fixed joints from the given KDL tree*/
    std::vector<std::string> jointNamesFromTree(const KDL::Tree &tree) const;

    /** Return full tree (KDL model)*/
    KDL::Tree getTree(){return full_tree;}

};

}

#endif // KINEMATICMODEL_HPP
