#ifndef KINEMATICCHAINKDL_HPP
#define KINEMATICCHAINKDL_HPP

#include <base/samples/Joints.hpp>
#include <map>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "TaskFrameKDL.hpp"

namespace wbc{

typedef std::map<std::string, uint> JointIndexMap;

class KinematicChainKDL{
public:
    KinematicChainKDL(const KDL::Chain& chain);
    ~KinematicChainKDL();

    void update(const base::samples::Joints &status);

    KDL::Chain chain_;
    KDL::JntArray q_, q_dot_, q_dot_dot_;

    std::vector<std::string> joint_names_; /** Names of joints in the kinematic chain */
    KDL::ChainFkSolverPos_recursive* pos_fk_solver_;
    KDL::ChainJntToJacSolver* jac_solver_;

    TaskFrameKDL* tf;
};
}

#endif // KINEMATICCHAINKDL_HPP
