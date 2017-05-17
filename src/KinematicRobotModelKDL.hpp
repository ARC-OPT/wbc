#ifndef KINEMATICROBOTMODELKDL_HPP
#define KINEMATICROBOTMODELKDL_HPP

#include "RobotModel.hpp"

#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class KinematicChainKDL;

/** This kinematic model describes the kinemetic relationships required for velocity based wbc. It is based on a single KDL Tree. However, multiple KDL trees can be added
 *  and will be appropriately concatenated. This way you can describe e.g. geometric robot-object relationships or create multi-robot scenarios.
 */
class KinematicRobotModelKDL : public RobotModel{

    typedef std::map<std::string, int> JointIndexMap;
    typedef std::map<std::string, KinematicChainKDL*> KinematicChainKDLMap;

protected:
    KDL::Tree full_tree;
    JointIndexMap joint_index_map;
    std::vector<std::string> joint_names;
    base::samples::Joints current_joint_state;
    std::vector<base::samples::RigidBodyState> current_poses;
    KinematicChainKDLMap kdl_chain_map;
    KDL::Jacobian robot_jacobian;
    base::Time last_update;

    void createJointIndexMap(const std::vector<std::string> &joint_names);
    void createChain(const std::string &root_frame, const std::string &tip_frame);
    void clear();
    std::vector<std::string> jointNamesFromTree(KDL::Tree tree);

public:
    /** The joint_names parameter defines the order of joints within the model, e.g. the column order in the Jacobians. If left empty, the
     *  joint order will be the same as in the overall KDL::Tree, which is alphabetical.
     */
    KinematicRobotModelKDL(const std::vector<std::string> &joint_names = std::vector<std::string>());
    virtual ~KinematicRobotModelKDL();

    /** Add a KDL Tree to the model. If the model is empty, the overall KDL::Tree will be replaced by the given tree. If there
     *  is already a KDL Tree, the new tree will be attached with the given pose to the hook frame of the overall tree. The relative poses
     *  of the trees can be updated online by calling update() with poses parameter appropriately set. This will also create the
     *  joint index map*/
    void addTree(const KDL::Tree& tree, const std::string& _hook = "", const KDL::Frame &pose = KDL::Frame::Identity());

    /**
     * Update the robot model. The joint state has to contain all joints that are relevant in the model. This means: All joints that are ever required
     * when requesting rigid body states, Jacobians or joint states. Note that
     *
     * @param joint_state The joint_state vector. Has to contain all robot joints.
     * @param poses Optionally update links of the robot model. This can be used to update e.g. the relative position between two robots in the model.
     */
    virtual void update(const base::samples::Joints& joint_state,
                        const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>());

    /** Returns the relative transform between the two given frames. By convention this is the pose of the tip frame in root coordinates!*/
    virtual void rigidBodyState(const std::string &root_frame,
                                const std::string &tip_frame,
                                base::samples::RigidBodyState& rigid_body_state);

    /** Returns the current status of the given joint names */
    virtual void jointState(const std::vector<std::string> &joint_names,
                            base::samples::Joints& joint_state);

    /** Returns the Jacobian for the kinematic chain between root and the tip frame. By convention the Jacobian is computed with respect to
        the root frame with the rotation point at the tip frame*/
    virtual void jacobian(const std::string &root_frame,
                          const std::string &tip_frame,
                          base::MatrixXd& jacobian);

    /** Check if a frame is available in the model*/
    bool hasFrame(const std::string &name);

protected:

};

}

#endif // KINEMATICMODEL_HPP
