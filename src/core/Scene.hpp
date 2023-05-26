#ifndef Scene_HPP
#define Scene_HPP

#include "TaskStatus.hpp"
#include "Constraint.hpp"
#include "QuadraticProgram.hpp"
#include "RobotModel.hpp"
#include "QPSolver.hpp"
#include "SceneConfig.hpp"

namespace wbc{

/**
 * @brief Base class for all wbc scenes.
 */
class Scene{
protected:
    RobotModelPtr robot_model;
    QPSolverPtr solver;
    std::vector< std::vector<TaskPtr> > tasks;
    std::vector< std::vector<ConstraintPtr> > constraints;
    TasksStatus tasks_status;
    HierarchicalQP hqp;
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
    Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double dt);
    ~Scene();

    /**
     * @brief Configure the WBC scene. Create tasks and sort them by priority
     * @param config configuration. Size has to be > 0. All tasks have to be valid. See TaskConfig.hpp for more details.
     */
    virtual bool configure(const std::vector<TaskConfig> &config);

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
     * @param activation Activation value. Has to be in interval [0.0Scene,1.0]
     */
    void setTaskActivation(const std::string& task_name, double activation);
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
    const TasksStatus& getTasksStatus() const { return tasks_status; }

    /**
     * @brief Sort task config by the priorities of the tasks
     */
    static void sortTaskConfig(const std::vector<TaskConfig>& config, std::vector< std::vector<TaskConfig> >& sorted_config);

    /**
     * @brief Return number of tasks per priority, given the task config
     */
    static std::vector<int> getNTaskVariablesPerPrio(const std::vector<TaskConfig>& config);

    /**
     * @brief updateTasksStatus Evaluate the fulfillment of the tasks given the current robot state and the solver output
     */
    virtual const TasksStatus& updateTasksStatus() = 0;

    /**
     * @brief Return tasks sorted by priority for the solver
     */
    void getHierarchicalQP(HierarchicalQP& _hqp){_hqp = hqp;}

    /**
     * @brief Get current solver output
     */
    const base::commands::Joints& getSolverOutput() const { return solver_output_joints; }

    /**
     * @brief set Joint weights by given name
     */
    void setJointWeights(const JointWeights& weights);

    /**
     * @brief Get Joint weights as Named vector
     */
    const JointWeights& getJointWeights() const { return joint_weights; }

    /**
     * @brief Get Joint weights as Named vector
     */
    const JointWeights& getActuatedJointWeights() const { return actuated_joint_weights; }

    /**
     * @brief Return the current robot model
     */
    RobotModelPtr getRobotModel(){return robot_model;}

    /**
     * @brief Return the current solver
     */
    QPSolverPtr getSolver(){return solver;}

    /**
     * @brief Return task configuration
     */
    std::vector<TaskConfig> getWbcConfig(){return wbc_config;}

};

typedef std::shared_ptr<Scene> ScenePtr;

template<typename T,typename RobotModelPtr,typename QPSolverPtr> Scene* createT(RobotModelPtr robot_model, QPSolverPtr solver, const double dt){return new T(robot_model,solver,dt);}

struct SceneFactory{
    typedef std::map<std::string, Scene*(*)(RobotModelPtr, QPSolverPtr, double)> SceneMap;

    static Scene *createInstance(const std::string& name, RobotModelPtr robot_model, QPSolverPtr solver, const double dt) {
        SceneMap::iterator it = getSceneMap()->find(name);
        if(it == getSceneMap()->end())
            throw std::runtime_error("Failed to create instance of plugin " + name + ". Is the plugin registered?");
        return it->second(robot_model, solver, dt);
    }

    template<typename T>
    static T* createInstance(const std::string& name, RobotModelPtr robot_model, QPSolverPtr solver, const double dt){
        Scene* tmp = createInstance(name, robot_model, solver, dt);
        T* ret = dynamic_cast<T*>(tmp);
        return ret;
    }

    static SceneMap *getSceneMap(){
        if(!scene_map)
            scene_map = new SceneMap;
        return scene_map;
    }

    static void clear(){
        scene_map->clear();
    }

private:
    static SceneMap *scene_map;
};

template<typename T>
struct SceneRegistry : SceneFactory{
    SceneRegistry(const std::string& name) {
        SceneMap::iterator it = getSceneMap()->find(name);
        if(it != getSceneMap()->end())
            throw std::runtime_error("Failed to register plugin with name " + name + ". A plugin with the same name is already registered");

        getSceneMap()->insert(std::make_pair(name, &createT<T,RobotModelPtr,QPSolverPtr>));
    }
};

} // namespace wbc

#endif
