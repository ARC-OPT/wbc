#ifndef KINMATICCHAINKDL_HPP
#define KINMATICCHAINKDL_HPP

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <base/Eigen.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace KDL {
class ChainFkSolverPos_recursive;
class ChainJntToJacSolver;
}

/** Helper class for storing information of a KDL chain in the robot model*/
namespace wbc{

class KinematicChainKDL{
public:
    KinematicChainKDL(KDL::Chain chain);
    ~KinematicChainKDL();

    /** Update joints and links of the kinematic chain*/
    void update(const base::samples::Joints& joint_state,
                const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>());

    KDL::Frame pose_kdl;
    base::samples::RigidBodyState rigid_body_state;
    KDL::Chain chain;
    KDL::JntArray joint_positions;
    KDL::Jacobian jacobian;
    std::vector<std::string> joint_names;
};

} // namespace wbc
#endif
