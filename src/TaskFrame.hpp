#ifndef TASK_FRAME_HPP
#define TASK_FRAME_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <base/samples/joints.h>
#include <kdl/chainjnttojacsolver.hpp>

namespace wbc{

class TaskFrame{
public:
    TaskFrame(const KDL::Chain& chain, const uint no_robot_joints);
    ~TaskFrame();

    void update(const base::samples::Joints &status);

    KDL::Chain chain_;
    KDL::Jacobian jac_;  /** Jacobian of kinematic chain expressed in root frame and with ref point in root frame */
    KDL::Jacobian jac_robot_;  /** Full robot Jacobian, filled only with information related to this task frame */
    KDL::Frame pose_; /** Full pose of tip frame of kinematic chain expressed in root frame */
    KDL::JntArray q_, q_dot_, q_dot_dot_;

    std::vector<std::string> joint_names_; /** Names of joints in the kinematic chain */
    KDL::ChainFkSolverPos_recursive* pos_fk_solver_;
    KDL::ChainJntToJacSolver* jac_solver_;

    std::vector<uint> joint_index_vector_;
};
}
#endif
