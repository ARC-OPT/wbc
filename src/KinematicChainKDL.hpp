#ifndef KINEMATICCHAINKDL_HPP
#define KINEMATICCHAINKDL_HPP

#include <base/samples/Joints.hpp>
#include <map>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include "TaskFrameKDL.hpp"
#include <kdl/jntspaceinertiamatrix.hpp>

namespace wbc{

typedef std::map<std::string, uint> JointIndexMap;

class KinematicChainKDL{
public:
    KinematicChainKDL(const KDL::Chain& chain);
    virtual ~KinematicChainKDL();

    virtual void update(const base::samples::Joints &status);

    KDL::Chain chain_;
    KDL::JntArray q_, q_dot_, q_dot_dot_;
    KDL::JntArray jnt_gravity_, jnt_coriolis_;

    std::vector<std::string> joint_names_; /** Names of joints in the kinematic chain */
    KDL::ChainFkSolverPos_recursive* pos_fk_solver_;
    KDL::ChainJntToJacSolver* jac_solver_;

    TaskFrameKDL* tf;
};

class KinematicChainKDLDyn : public KinematicChainKDL{
public:
    KinematicChainKDLDyn(const KDL::Chain& chain, const Eigen::Vector3d &gravity);
    ~KinematicChainKDLDyn();

    virtual void update(const base::samples::Joints &status);

    KDL::JntArray jnt_gravity_, jnt_coriolis_;
    KDL::ChainDynParam* dyn_param_solver_;
    KDL::JntSpaceInertiaMatrix jnt_inertia_;
};
}

#endif // KINEMATICCHAINKDL_HPP
