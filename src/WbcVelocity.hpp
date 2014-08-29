#ifndef WBC_HPP
#define WBC_HPP

#include "ConstraintConfig.hpp"
#include <base/commands/joints.h>
#include "TaskFrame.hpp"
#include "SolverTypes.hpp"
#include <map>

namespace wbc{

class ExtendedConstraint;
class Constraint;

/**
 * @brief The WbcVelocity class creates the equation system for the solver. It retrieves task frames that contain information about poses and Jacobians
 *        of task related robot coordinate system, e.g. the end effector.
 */

class WbcVelocity{

    typedef std::map<std::string, ExtendedConstraint*> ConstraintMap;
    typedef std::map<std::string, uint> JointIndexMap;
    typedef std::map<std::string, TaskFrameKDL> TaskFrameKDLMap;
    typedef std::map<std::string, KDL::Jacobian> RobotJacobianMap;

protected:
    void clear();
    bool configured_;
    ConstraintMap constraint_map_; /** Map associating names of Constraints to Constraint pointers */
    std::vector< std::vector<ExtendedConstraint*> > constraint_vector_;  /** Vector containing all constraints (ordered by priority, highest priority first) */
    double constraint_timeout_; /** In seconds. A constraint will be deactivated if no new reference comes in for such a long time. If set to .nan, no timeout will be used. */
    uint no_robot_joints_;
    JointIndexMap joint_index_map_; /** Maps joint names to indices. Order will be either as in KDL::Tree, or as in joint_names parameter given to configure() */
    TaskFrameKDLMap tf_map_;   /** Map of task frames, will be created with the first call of configure() */
    RobotJacobianMap jac_map_; /** Full robot jacobians, order of joints will be as defined in joint_names (see configure()*/

    //Helpers
    Eigen::VectorXd temp_;
    bool has_timeout_;
    KDL::Twist tw_;

    SolverInput solver_input_;

public:
    /**
     * @brief Create Robot Model and Solver
     * @param tree Kinematic tree for the robot model
     * @param type Which wbc to use?
     */
    WbcVelocity();
    ~WbcVelocity();

    /**
     * @brief Configure Has to be called before calling prepareEqSystem for the first time.
     * @param config Constraints configuration vector. Contains information about the constraints of the overall task.
     * @param joint_names Order of joint names that shall be used internally. Has to contain all joints that are involved in any of the constraints.
     * @param constraints_active constraints will be active in the beginning, if this is set to true
     * @param constraints_timeout In seconds. Constraints will be timed out after this amount of time, their weights and inout will be zero, so that they do
     *                     not disturb the active constraints anymore. If constraint_timeout == .nan no timeout will be used.
     * @return True in case of success, false else
     */
    bool configure(const std::vector<ConstraintConfig> &config,
                   const std::vector<std::string> &joint_names,
                   bool constraints_active,
                   double constraint_timeout);

    /**
     * @brief solve Compute control solution given all the constraints
     * @param status Joint status. Has to contain at least all joints that have been configured, i.e. all joints that are located between the robot root from the given KDL::Tree and
     *               and all task frames. Order may be arbitrary. The joints will be mapped correctly according to the given names.
     * @param ctrl_out Control output. Size will be same as total no of joints as returned by noOfJoints()
     */
    void prepareEqSystem(const std::vector<TaskFrame> &task_frames, std::vector<ConstraintsPerPrio> &constraints);

    uint noOfJoints(){return no_robot_joints_;}
    Constraint* constraint(const std::string &name);
    std::vector<std::string> jointNames();
    uint jointIndex(const std::string &joint_name){return joint_index_map_[joint_name];}

};
}
#endif

