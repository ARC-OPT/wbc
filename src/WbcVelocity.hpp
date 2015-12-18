#ifndef WBC_HPP
#define WBC_HPP

#include "Constraint.hpp"
#include "LinearEquationSystem.hpp"
#include "TaskFrame.hpp"

namespace wbc{

class ExtendedConstraint;

/**
 * @brief The WbcVelocity class creates the equation system for the solver. It retrieves task frames that contain information about poses and Jacobians
 *        of task related robot coordinate system, e.g. the end effector.
 */

class WbcVelocity{

    typedef std::map<std::string, ExtendedConstraint*> ConstraintMap;
    typedef std::map<std::string, uint> JointIndexMap;
    typedef std::map<std::string, KDL::Jacobian> RobotJacobianMap;

protected:
    void clear();
    bool configured_;
    ConstraintMap constraint_map_; /** Map associating names of Constraints to Constraint pointers */
    std::vector< std::vector<ExtendedConstraint*> > constraint_vector_;  /** Vector containing all constraints (ordered by priority, highest priority first) */
    std::vector<int> n_constraints_per_prio_; /** Number of constraint variables per priority*/
    uint n_prios_; /** Number of priprities */
    uint no_robot_joints_; /** Number of configured robot joints*/
    JointIndexMap joint_index_map_; /** Maps joint names to indices. Order will be either as in KDL::Tree, or as in joint_names parameter given to configure() */
    RobotJacobianMap jac_map_; /** Full robot jacobians, order of joints will be as defined in joint_names (see configure()*/
    std::vector<std::string> task_frame_ids;

    //Helpers
    Eigen::VectorXd temp_;
    bool has_timeout_;
    KDL::Twist tw_;
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
                   const std::vector<std::string> &joint_names);

    /**
     * @brief solve Compute control solution given all the constraints
     * @param status Joint status. Has to contain at least all joints that have been configured, i.e. all joints that are located between the robot root from the given KDL::Tree and
     *               and all task frames. Order may be arbitrary. The joints will be mapped correctly according to the given names.
     * @param ctrl_out Control output. Size will be same as total no of joints as returned by noOfJoints()
     */
    void prepareEqSystem(const TaskFrameMap &task_frames,
                         std::vector<LinearEquationSystem> &equations);

    uint noOfJoints(){return no_robot_joints_;}
    Constraint* constraint(const std::string &name);
    std::vector<std::string> jointNames();
    uint jointIndex(const std::string &joint_name){return joint_index_map_[joint_name];}
    std::vector<std::string> getTaskFrameIDs(){return task_frame_ids;}
    std::vector<int> getNumberOfConstraintsPP(){return n_constraints_per_prio_;}
    void getConstraintVector(std::vector<ConstraintsPerPrio>& constraints);

};
}
#endif

