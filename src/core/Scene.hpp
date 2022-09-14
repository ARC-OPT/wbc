#ifndef WBCSCENE_HPP
#define WBCSCENE_HPP

#include "TaskStatus.hpp"
#include "Constraint.hpp"
#include "QuadraticProgram.hpp"
#include "RobotModel.hpp"
#include "QPSolver.hpp"

namespace wbc{

/**
 * @brief Base class for all wbc scenes.
 */
class WbcScene{
protected:
    RobotModelPtr robot_model;
    QPSolverPtr solver;
    std::vector< std::vector<TaskPtr> > tasks;
    std::vector< std::vector<ConstraintPtr> > constraints;
    TasksStatus tasks_status;
    HierarchicalQP tasks_prio;
    std::vector<int> n_task_variables_per_prio;
    bool configured;
    base::commands::Joints solver_output_joints;
    JointWeights joint_weights, actuated_joint_weights;
    std::vector<TaskConfig> wbc_config;

    /**
     * brief Create a task and add it to the WBC scene
     */
    virtual TaskPtr createTask(const TaskConfig &config) = 0;

    /**
     * @brief Delete all tasks and free memory
     */
    void clearTasks();

public:
    WbcScene(RobotModelPtr robot_model, QPSolverPtr solver);
    ~WbcScene();
    /**
     * @brief Configure the WBC scene. Create tasks and sort them by priority
     * @param config configuration. Size has to be > 0. All tasks have to be valid. See TaskConfig.hpp for more details.
     */
    bool configure(const std::vector<TaskConfig> &config);

    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @return Hierarchical quadratic program (solver input)
     */
    virtual const HierarchicalQP& update() = 0;

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint command
     */
    virtual const base::commands::Joints& solve(const HierarchicalQP& hqp) = 0;

    /**
     * @brief Set reference input for a joint space task
     * @param task_name Name of the task
     * @param task_name Joint space reference values
     */
    void setReference(const std::string& task_name, const base::samples::Joints& ref);

    /**
     * @brief Set reference input for a cartesian space task
     * @param task_name Name of the task
     * @param task_name Cartesian space reference values
     */
    void setReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref);

    /**
     * @brief Set Task weights input for a  task
     * @param task_name Name of the task
     * @param weights Weight vector. Size has to be same as number of task variables
     */
    void setTaskWeights(const std::string& task_name, const base::VectorXd &weights);
    /**
     * @brief Set Task activation for a  task
     * @param task_name Name of the task
     * @param activation Activation value. Has to be in interval [0.0,1.0]
     */
    void setTaskActivation(const std::string& task_name, const double activation);
    /**
     * @brief Return a Particular task. Throw if the task does not exist
     */
    TaskPtr getTask(const std::string& name);

    /**
     * @brief True in case the given task exists
     */
    bool hasTask(const std::string& name);

    /**
     * @brief Returns all tasks as vector
     */
    const TasksStatus& getTasksStatus(){return tasks_status;}

    /**
     * @brief Sort task config by the priorities of the tasks
     */
    static void sortTaskConfig(const std::vector<TaskConfig>& config, std::vector< std::vector<TaskConfig> >& sorted_config);

    /**
     * @brief Return number of tasks per priority, given the task config
     */
    static std::vector<int> getNTaskVariablesPerPrio(const std::vector<TaskConfig> &config);

    /**
     * @brief updateTasksStatus Evaluate the fulfillment of the tasks given the current robot state and the solver output
     */
    virtual const TasksStatus &updateTasksStatus() = 0;

    /**
     * @brief Return tasks sorted by priority for the solver
     */
    void getHierarchicalQP(HierarchicalQP& hqp){hqp = tasks_prio;}

    /**
     * @brief Get current solver output
     */
    const base::commands::Joints& getSolverOutput(){return solver_output_joints;}

    /**
     * @brief set Joint weights by given name
     */
    void setJointWeights(const JointWeights &weights);

    /**
     * @brief Get Joint weights as Named vector
     */
    const JointWeights &getJointWeights(){return joint_weights;}

    /**
     * @brief Get Joint weights as Named vector
     */
    const JointWeights &getActuatedJointWeights(){return actuated_joint_weights;}

    /**
     * @brief Return the current robot model
     */
    RobotModelPtr getRobotModel(){return robot_model;}

    /**
     * @brief Return the current solver
     */
    QPSolverPtr getSolver(){return solver;}

    std::vector<TaskConfig> getWbcConfig(){return wbc_config;}


    /**  LEGACY METHODS, DEPRECATED, SOON TO BE REMOVED*/

    [[deprecated("Renamed Constraint to Task: called setTaskWeights instead")]]
    void setConstraintWeights(const std::string& constraint_name, const base::VectorXd &weights) { 
        setTaskWeights(constraint_name, weights); 
    }

    [[deprecated("Renamed Constraint to Task: called setTaskActivation instead")]]
    void setConstraintActivation(const std::string& constraint_name, const double activation) { 
        setTaskActivation(constraint_name, activation); 
    }

    [[deprecated("Renamed Constraint to Task: called getTask instead")]]
    TaskPtr getConstraint(const std::string& name) { 
        return getTask(name); 
    }

    [[deprecated("Renamed Constraint to Task: called hasTask instead")]]
    bool hasConstraint(const std::string& name) { 
        return hasTask(name); 
    }

    [[deprecated("Renamed Constraint to Task: called getTasksStatus instead")]]
    const TasksStatus& getConstraintsStatus(){
        return getTasksStatus(); 
    }

    [[deprecated("Renamed Constraint to Task: called sortTaskConfig instead")]]
    static void sortConstraintConfig(const std::vector<TaskConfig>& config, std::vector< std::vector<TaskConfig> >& sorted_config) {
        sortTaskConfig(config, sorted_config); 
    }

    [[deprecated("Renamed Constraint to Task: called getNTaskVariablesPerPrio instead")]]
    static std::vector<int> getNConstraintVariablesPerPrio(const std::vector<TaskConfig> &config) { 
        return getNTaskVariablesPerPrio(config); 
    }

    [[deprecated("Renamed Constraint to Task: called updateTasksStatus instead")]]
    const TasksStatus &updateConstraintsStatus() { return this->updateTasksStatus(); }

};

typedef std::shared_ptr<WbcScene> WbcScenePtr;

} // namespace wbc

#endif
