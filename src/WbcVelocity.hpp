#ifndef WBCVELOCITY_HPP
#define WBCVELOCITY_HPP

#include "Wbc.hpp"
#include "constraints/ExtendedConstraint.hpp"
#include <map>

namespace wbc{

/**
 * @brief The WbcVelocity class implements a velocity based WBC. It used KDL for the kinematics computations.
 */
class WbcVelocity : public Wbc{

    typedef std::map<std::string, ExtendedConstraint*> ConstraintMap;
    typedef std::map<std::string, uint> JointIndexMap;
    typedef std::map<std::string, KDL::Jacobian> RobotJacobianMap;
    typedef std::map<std::string, TaskFrame*> TaskFrameMap;

protected:
    void clear();
    bool configured_;
    ConstraintMap constraint_map_; /** Map associating names of Constraints to Constraint pointers */
    std::vector<int> n_constraints_per_prio_; /** Number of constraint variables per priority*/
    JointIndexMap joint_index_map_; /** Maps joint names to indices. Order will be either as in KDL::Tree, or as in joint_names parameter given to configure() */
    RobotJacobianMap jac_map_; /** Full robot jacobians, order of joints will be as defined in joint_names (see configure()*/

    //Helpers
    Eigen::VectorXd temp_;
public:
    WbcVelocity();
    ~WbcVelocity();

    /**
     * @brief Interface method for configuring WBC. Implement this in your concrete WBC implementation (velocity-based WBC, force-based WBC).
     *        This should prepare all required data containers given the constraint configuration and the order of the joints.
     * @param config Constraint configuration for WBC. Can have arbitrary size > 0.
     * @param joint_names This defines the order of joints within all matrices and vectors (Jacobians, weight matrices, etc.)
     * @return true in case of success, false in case of failure
     */
    virtual bool configure(const std::vector<ConstraintConfig> &config,
                           const std::vector<std::string> &joint_names);

    /**
     * @brief setupEquationSystem Given all required task frames, this will setup the equation system for the constraint solver
     * @param task_frames A map of task
     * @param equations
     */
    virtual void setupOptProblem(const std::vector<TaskFrame*> &task_frames, std::vector<OptProblem*> &opt_problem);
    void evaluateConstraints(const base::VectorXd& solver_output,
                             const base::VectorXd& robot_vel);

};
}
#endif

