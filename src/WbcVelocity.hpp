#ifndef WBC_HPP
#define WBC_HPP

#include "ConstraintConfig.hpp"
#include <kdl/tree.hpp>
#include <base/commands/joints.h>
#include <Eigen/Core>
#include <kdl/chainjnttojacsolver.hpp>
#include "HierarchicalWDLSSolver.hpp"

namespace wbc{

class TaskFrame;
class ExtendedConstraint;
class Constraint;

typedef std::map<std::string, ExtendedConstraint*> ConstraintMap;
typedef std::map<std::string, int> JointIndexMap;
typedef std::map<std::string, TaskFrame*> TaskFrameMap;

class WbcVelocity{
protected:
    bool addTaskFrame(const std::string &frame_id);
    void clear();

    HierarchicalWDLSSolver* solver_;

    bool debug_;
    bool configured_;
    KDL::Tree tree_;
    std::string robot_root_;
    ConstraintMap constraint_map_; /** Map associating names of Constraints to Constraint pointers */
    std::vector< std::vector<ExtendedConstraint*> > constraint_vector_;  /** Vector containing all constraints (ordered by priority, highest priority first) */
    TaskFrameMap task_frame_map_; /** Map containing all task frames, that are associated with the cartesian constraints*/
    double constraint_timeout_; /** In seconds. A constraint will be deactivated if no new reference comes in for such a long time. If set to .nan, no timeout will be used. */
    uint no_robot_joints_;
    JointIndexMap joint_index_map_; /** Maps joint names to indices. Order will be either as in KDL::Tree, or as in joint_names parameter given to configure() */

    //Helpers
    Eigen::VectorXd temp_;
    bool has_timeout_;
    KDL::Twist tw_;

    std::vector<SolverInput> solver_input_;

public:
    /**
     * @brief Create Robot Model and Solver
     * @param tree Kinematic tree for the robot model
     * @param type Which wbc to use?
     */
    WbcVelocity();
    ~WbcVelocity();

    /**
     * @brief Configure Has to be called before calling solve for the first time.
     * @param tree KDL Tree describing the robot
     * @param config Constraints configuration vector.
     * @param joint_names Optional order of joint names that shall be used internally. If empty the order from KDL::Tree will be used (alphabetic)
     * @param constraints_active constraints will be active in the beginning, if this is set to true
     * @param constraints_timeout In seconds. Constraints will be timed out after this amount of time, their weights and inout will be zero, so that they do
     *                     not disturb the active constraints anymore. If constraint_timeout == .nan no timeout will be used.
     * @return True in case of success, false else
     */
    bool configure(const KDL::Tree tree,
                   const std::vector<ConstraintConfig> &config,
                   const std::vector<std::string> &joint_names,
                   bool constraints_active,
                   double constraint_timeout,
                   bool debug = false);

    /**
     * @brief solve Compute control solution given all the constraints
     * @param status Joint status. Has to contain at least all joints that have been configured, i.e. all joints that are located between the robot root from the given KDL::Tree and
     *               and all task frames. Order may be arbitrary. The joints will be mapped correctly according to the given names.
     * @param ctrl_out Control output. Size will be same as total no of joints as returned by noOfJoints()
     */
    void solve(const base::samples::Joints &status, Eigen::VectorXd &ctrl_out);

    uint noOfJoints(){return no_robot_joints_;}
    Constraint* constraint(const std::string &name);
    HierarchicalWDLSSolver* solver(){return solver_;}
    std::vector<std::string> jointNames();
    uint jointIndex(const std::string &joint_name){return joint_index_map_[joint_name];}

};
}
#endif

