#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include "RobotModel.hpp"

#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "Jacobian.hpp"

namespace wbc{

class RobotModelConfig;
class KinematicChainKDL;

/**
 *  @brief This model describes the kinemetic relationships required for velocity based wbc. It is based on a single KDL Tree. However, multiple KDL trees can be added
 *  and will be appropriately concatenated. This way you can describe e.g. geometric robot-object relationships or create multi-robot scenarios.
 */
class KinematicRobotModelKDL : public RobotModel{

    typedef std::map<std::string, KinematicChainKDL*> KinematicChainKDLMap;
    typedef std::map<std::string, Jacobian> JacobianMap;

protected:
    KDL::Tree full_tree;                                       /** Overall kinematic tree*/
    base::samples::Joints current_joint_state;                 /** Last joint state that was passed through the call of update()*/
    std::vector<base::samples::RigidBodyState> current_poses;  /** Last poses that were passed through the call of update()*/
    KinematicChainKDLMap kdl_chain_map;                        /** Map of KDL Chains. Entries are generated through calls of rigidBodyState() or jacobian()*/
    JacobianMap jac_map;                                       /** Map of robot jacobians for all kinematic chains*/
    base::samples::Joints joint_state;                         /** Helper variable*/

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
    bool addTree(const KDL::Tree& tree, const std::string& hook = "", const base::samples::RigidBodyState &pose = base::samples::RigidBodyState());

    /** Free storage and clear data structures*/
    void clear();

public:
    KinematicRobotModelKDL();
    virtual ~KinematicRobotModelKDL();

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
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>());

    /**
     * @brief Computes and returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates.
     *  This will create a kinematic chain between root and tip frame, if called for the first time with the given arguments.
     * @param root_frame Root frame of the chain. Has to be a valid link in the robot model.
     * @param tip_frame Tip frame of the chain. Has to be a valid link in the robot model.
     */
    virtual const base::samples::RigidBodyState &rigidBodyState(const std::string &root_frame, const std::string &tip_frame);

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

    /** Check if a frame is available in the model*/
    bool hasFrame(const std::string &name);

    /** Return all non-fixed joints from the given KDL tree*/
    std::vector<std::string> jointNamesFromTree(const KDL::Tree &tree) const;

    KDL::Tree getTree(){return full_tree;}
};

}

#endif // KINEMATICMODEL_HPP
