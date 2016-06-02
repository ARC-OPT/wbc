#ifndef WBC_HPP
#define WBC_HPP

#include "constraints/ConstraintConfig.hpp"

namespace wbc{

class Constraint;
class OptProblem;
class TaskFrame;

/**
 * @brief The Wbc class represents the interface between the robot model and the solver. It retrieves the required information
 *        about the current state of kinematics and dynamics of the robot from the robot model and, based on this information,
 *        prepares the equation system for the solver.
 */
class Wbc{
protected:
    std::vector< std::vector<Constraint*> > constraints;
    std::vector<std::string> joint_names;

public:
    Wbc();
    virtual ~Wbc();

    /**
     * @brief Interface method for configuring WBC. Implement this in your concrete WBC implementation (velocity-based WBC, force-based WBC).
     *        This should prepare all required data containers given the constraint configuration and the order of the joints.
     * @param config Constraint configuration for WBC. Can have arbitrary size > 0.
     * @param joint_names This defines the order of joints within all matrices and vectors (Jacobians, weight matrices, etc.)
     * @return true in case of success, false in case of failure
     */
    virtual bool configure(const std::vector<ConstraintConfig> &config,
                           const std::vector<std::string> &joint_names) = 0;

    /**
     * @brief setupEquationSystem Given all required task frames, this will setup the equation system for the constraint solver
     * @param task_frames A map of task
     * @param equations
     */
    virtual void setupOptProblem(const std::vector<TaskFrame*> &task_frames, std::vector<OptProblem*> &opt_problem) = 0;

    /**
     * @brief Return a Particular constraint
     */
    Constraint* getConstraint(const std::string& name);

    /**
     * @brief Return all constraints as vector
     */
    std::vector< std::vector<Constraint*> > getConstraints(){return constraints;}

    /**
     * @brief Return all required task frame ids by iterating through all constraints
     */
    std::vector<std::string> getTaskFrameIDs();

    /**
     * @brief Return all joint names
     */
    const std::vector<std::string> &getJointNames(){return joint_names;}

    /**
     * @brief Returns the number of constraint variables per priority
     */
    std::vector<int> getNumberOfConstraintsPerPriority();
};
}

#endif
