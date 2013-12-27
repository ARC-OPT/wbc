#ifndef SUBTASK_HPP
#define SUBTASK_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/**
 * @brief Defines a sub task in the whole body control problem. Each sub task is characterized by a kinematic chain
 */
class SubTask{
public:
    SubTask(const KDL::Chain &chain, const uint no_task_variables, const uint priority);
    ~SubTask();

    /**
     * @brief Update kinematic and dynamic equations
     * @param q Joint angles of all joints in the kinematic chain in rad
     * @param q_dot Joint velocities of all joints in the kinematic chain in rad/sec
     * @param q_dot_dot Joint accelerations of all joints in the kinematic chain in rad/ssec
     */
    void Update();

    KDL::Chain chain_; /** Kinematic chain that describes this sub task */
    KDL::ChainFkSolverPos_recursive* pos_fk_solver_;
    KDL::ChainJntToJacSolver* jac_solver_;

    KDL::Jacobian jac_;  /** Jacobian of kinematic chain expressed in root frame and with ref point in root frame */
    KDL::Frame pose_; /** Full pose of tip frame of kinematic chain expressed in root frame */
    KDL::JntArray q_, q_dot_, q_dot_dot_;
    uint no_task_variables_;
    uint priority_;
};

#endif
