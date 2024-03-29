#ifndef KINMATICCHAINKDL_HPP
#define KINMATICCHAINKDL_HPP

#include <memory>
#include <kdl/frameacc.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <base/Time.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <map>

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
    base::samples::RigidBodyStateSE3 cartesian_state;

public:
    KinematicChainKDL(const KDL::Chain &chain, const std::string &root_frame, const std::string &tip_frame);

    /**
     * @brief Update all joints of the kinematic chain
     * @param joint_state Has to contain at least all joints that are included in the kinematic chain. Each entry has to have a valid position, velocity and acceleration
     */
    void update(const KDL::JntArray& q, const KDL::JntArray& qd, const KDL::JntArray& qdd, std::map<std::string,int> joint_idx_map);
    /** Convert and return current Cartesian state*/
    const base::samples::RigidBodyStateSE3& rigidBodyState();

    /** Compute FK (pose, twist,spatial acc) for the chain using the current joint state. Note: This will call
     * calculateSpaceJacobian() and calculateJacobianDot() if space_jacobian and jacobian_dot are not up to date*/
    void calculateForwardKinematics();
    /** Compute space Jacobian using the current joint state*/
    void calculateSpaceJacobian();
    /** Compute body Jacobian using the current joint state. Note: This will call calculateSpaceJacobian() if space_jacobian is not up to date*/
    void calculateBodyJacobian();
    /** Compute derivative of space Jacobian (hybrid representation) using the current joint state*/
    void calculateJacobianDot();


    KDL::Frame pose_kdl;                             /** KDL Pose of the tip segment in root coordinate of the chain*/
    KDL::FrameVel frame_vel;                         /** Helper for the velocity fk*/
    KDL::Twist twist_kdl;                            /** KDL Pose of the tip segment in root coordinate of the chain*/
    base::Vector6d acc;                              /** Helper to store current frame acceleration*/
    KDL::Chain chain;                                /** The underlying KDL chain*/
    KDL::JntArrayVel jnt_array_vel;                  /** Vector of positions and velocities of all included joints*/
    KDL::JntArrayAcc jnt_array_acc;                  /** Vector of positions, velocities and accelerations of all included joints*/
    KDL::Jacobian space_jacobian;                    /** Space Jacobian of the Chain. Reference frame is root & reference point is tip*/
    KDL::Jacobian body_jacobian;                     /** Body Jacobian of the Chain. Reference frame is root & reference point is tip*/
    KDL::Jacobian jacobian_dot;                      /** Derivative of Jacobian of the Chain. Reference frame & reference point is the root frame*/
    std::vector<std::string> joint_names;            /** Names of the joint included in the kinematic chain*/
    std::string root_frame;                          /** UID of the kinematics chain root link*/
    std::string tip_frame;                           /** UID of the kinematics chain tip link*/
    base::Time stamp;
    bool has_acceleration, space_jacobian_is_up_to_date, body_jacobian_is_up_to_date, jac_dot_is_up_to_date, fk_is_up_to_date;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver;

};

} // namespace wbc
#endif
