#ifndef WBC_HPP
#define WBC_HPP

#include "ConstraintConfig.hpp"
#include "Constraint.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <base/commands/Joints.hpp>

namespace wbc{

class Constraint;
class OptProblem;
class TaskFrame;

/**
 * @brief The Wbc class represents the interface between the robot model and the solver. It retrieves the required information
 *        about the current state of kinematics and dynamics of the robot from the robot model and, based on this information,
 *        prepares the optimization problem for the solver.
 */
class Wbc{
protected:
    std::vector< std::vector<Constraint*> > constraints;
    std::vector< ConstraintsPerPrio > constraint_vector;

public:
    Wbc();
    virtual ~Wbc();

    /**
     * @brief Interface method for configuring WBC. This should prepare all required data containers given the constraint configuration and the order of the joints.
     * @param config Constraint configuration for WBC. Can have arbitrary size > 0.
     * @param joint_names This defines the order of joints within all matrices and vectors (Jacobians, weight matrices, etc.)
     * @return true in case of success, false in case of failure
     */
    virtual bool configure(const std::vector<ConstraintConfig> &config,
                           const std::vector<std::string> &joint_names) = 0;

    /**
     * @brief Given all required task frames, this will setup the equation system for the constraint solver
     * @param task_frames A map of task
     * @param equations
     */
    virtual void setupOptProblem(const std::vector<TaskFrame*> &task_frames, OptProblem &opt_problem) = 0;

    /**
     * @brief Update a joint space constraint.
     * @param name Name of the constraint
     * @param reference Reference input.
     */
    virtual void setReference(const std::string &name,
                              const base::commands::Joints& reference) = 0;

    /**
     * @brief Update a Cartesian space constraint
     * @param name Name of the constraint
     * @param reference Reference input.
     */
    virtual void setReference(const std::string &name,
                              const base::samples::RigidBodyState& reference) = 0;

    /**
     * @brief Update the weights of a particular constraint
     * @param weights Weight Vector. Size has to be same as number of constraint variables. Values have to be >= 0
     */
    void setConstraintWeights(const std::string &name,
                              const base::VectorXd& weights);

    /**
     * @brief Update the activation of a particular constraint
     * @param activation Activation value. Has to be >= 0 and <= 1
     */
    void setConstraintActivation(const std::string &name,
                                    const double activation);

    /**
     * @brief Return a Particular constraint. Throw if the constraint does not exist
     */
    Constraint* getConstraint(const std::string& name);

    /**
     * @brief hasConstraint True in case the given constraint exists
     */
    bool hasConstraint(const std::string& name);

    /**
     * @brief Return all constraints as vector
     */
    std::vector< ConstraintsPerPrio > getConstraints();

    /**
     * @brief Returns the number of constraint variables per priority
     */
    std::vector<int> getConstraintVariablesPerPrio();

    /**
     * @brief getTaskFrameIDs Given the WBC config, this methods returns all required task frames
     */
    std::vector<std::string> getTaskFrameIDs(const std::vector<ConstraintConfig> &config);

    /**
     * @brief getTaskFrameByName Helper Method that returns the task frame with the given name from the task frame vector.
     *                           Will throw if the task frame does not exists.
     */
    TaskFrame* getTaskFrameByName(const std::vector<TaskFrame*> task_frames, const std::string& tf_name);

    /**
     * @brief getNumberOfPriorities Returns number of priority levels
     */
    size_t getNumberOfPriorities(){return constraints.size();}

    /**
     * @brief sortConfigByPriority Sort WBC config by the priorities of the constraints
     * @param config input config
     * @param sorted_config output config
     */
    static void sortConfigByPriority(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config);

    /**
     * @brief isValid Check if the given WBC config is valid
     */
    static bool isValid(const std::vector<ConstraintConfig>& config);
};
}

#endif
