#ifndef KINMATICCHAINKDL_HPP
#define KINMATICCHAINKDL_HPP

#include <memory>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <base/Time.hpp>
#include <wbc_common/CartesianState.hpp>

namespace base{
    namespace samples{
        class Joints;
    }
}
namespace wbc{

/**
 * @brief Helper class for storing information of a KDL chain in the robot model
*/
class KinematicChainKDL{

protected:
    CartesianState cartesian_state;

public:
    KinematicChainKDL(const KDL::Chain &chain);

    /**
     * @brief Update all joints and links of the kinematic chain
     * @param joint_state Has to contain at least all joints that are included in the kinematic chain. Each entry has to have a valid position
     * @param poses If not empty, the method will try to update the pose of all links that match the sourceFrame of the given rigidBodyState.
     * Must have a valid position and orientation entry. All other entries will be ignored. The matching segment transform will be replaced by
     * the given pose in the rigid body state. This can be used e.g. to update relativ positions between multiple KDL tree in the model (i.e. the
     * kinematic chains can stretch over more than one tree like this
     */
    void update(const base::samples::Joints& joint_state);

    const CartesianState& cartesianState();

    base::Time last_update;                          /** Latest information change within this kinematic chain*/
    KDL::Frame pose_kdl;                             /** KDL Pose of the tip segment in root coordinate of the chain*/
    KDL::FrameVel frame_vel;                         /** Helper for the velocity fk*/
    KDL::Twist twist_kdl;                            /** KDL Pose of the tip segment in root coordinate of the chain*/
    KDL::Chain chain;                                /** The underlying KDL chain*/
    KDL::JntArrayVel joint_state_kdl;                /** Vector of positions of all included joints*/
    KDL::Jacobian jacobian;                          /** Jacobian of the Chain. Reference frame is root & reference point is tip*/
    KDL::Jacobian jacobian_ref_root;                 /** Jacobian of the Chain. Reference frame & reference point is root*/
    KDL::Jacobian jacobian_dot;                      /** Derivative of Jacobian of the Chain. Reference frame & reference point is the root frame*/
    std::vector<std::string> joint_names;            /** Names of the joint included in the kinematic chain*/
    std::vector<std::string> segment_names;          /** Names of all segments including root segment*/
};

} // namespace wbc
#endif
