#ifndef KINMATICCHAINKDL_HPP
#define KINMATICCHAINKDL_HPP

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <base/Eigen.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <kdl/jacobian.hpp>

namespace KDL {
class ChainFkSolverPos_recursive;
class ChainJntToJacSolver;
}

namespace wbc{

class KinematicChainKDL{
public:
    KinematicChainKDL(KDL::Chain chain);
    ~KinematicChainKDL();

    void update(const base::samples::Joints& joint_state,
                const std::vector<base::samples::RigidBodyState>& poses = std::vector<base::samples::RigidBodyState>());

    KDL::Frame pose_kdl;
    base::samples::RigidBodyState rigid_body_state;
    std::vector<std::string> joint_names;
    KDL::JntArray joint_positions;
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive *fk_solver;
    KDL::ChainJntToJacSolver *jac_solver;
    KDL::Jacobian jacobian;
};

} // namespace wbc
#endif
