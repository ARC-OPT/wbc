#ifndef WBC_CORE_SCENE_HPP
#define WBC_CORE_SCENE_HPP

#include "Task.hpp"
#include "Constraint.hpp"
#include "QuadraticProgram.hpp"
#include "RobotModel.hpp"
#include "QPSolver.hpp"
#include "../types/JointCommand.hpp"

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
    HierarchicalQP hqp;
    std::vector<int> n_task_variables_per_prio;
    bool configured;
    types::JointCommand solver_output_joints;
    Eigen::VectorXd joint_weights, actuated_joint_weights;
    Eigen::VectorXd solver_output;

    /**
     * @brief Delete all tasks and free memory
     */
    void clearTasks();

public:
    Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double dt);
    ~Scene();

    /**
     * @brief Configure the WBC scene. Create tasks and sort them by priority given the task config
     * @param tasks Tasks used in optimization function. Size has to be > 0. All tasks have to be valid. See tasks and TaskConfig.hpp for more details.
     */
    virtual bool configure(const std::vector<TaskPtr> &tasks);

    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @return Hierarchical quadratic program (solver input)
     */
    virtual const HierarchicalQP& update() = 0;

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint command
     */
    virtual const types::JointCommand& solve(const HierarchicalQP& hqp) = 0;

    /**
     * @brief Return a Particular task. Throw if the task does not exist
     */
    TaskPtr getTask(const std::string& name);

    /**
     * @brief True in case the given task exists
     */
    bool hasTask(const std::string& name);

    /**
     * @brief Sort task config by the priorities of the tasks
     */
    static void sortTasks(const std::vector<TaskPtr>& tasks, std::vector< std::vector<TaskPtr> >& sorted_tasks);

    /**
     * @brief Return number of tasks per priority, given the task config
     */
    static uint getNTaskVariables(const std::vector<TaskPtr>& config);

    /**
     * @brief Return number of tasks per priority, given the task config
     */
    static std::vector<int> getNTaskVariablesPerPrio(const std::vector<TaskPtr>& config);

    /**
     * @brief Return tasks sorted by priority for the solver
     */
    void getHierarchicalQP(HierarchicalQP& _hqp){_hqp = hqp;}

    /**
     * @brief Get current solver output
     */
    const types::JointCommand& getSolverOutput() const { return solver_output_joints; }

    /**
     * @brief Get current solver output in raw values
     */
    const Eigen::VectorXd& getSolverOutputRaw() const { return solver_output; }

    /**
     * @brief set Joint weights by given name
     */
    void setJointWeights(const Eigen::VectorXd& weights);

    /**
     * @brief Get Joint weights as Named vector
     */
    const Eigen::VectorXd& getJointWeights() const { return joint_weights; }

    /**
     * @brief Get Joint weights as Named vector
     */
    const Eigen::VectorXd& getActuatedJointWeights() const { return actuated_joint_weights; }

    /**
     * @brief Return the current robot model
     */
    RobotModelPtr getRobotModel(){return robot_model;}

    /**
     * @brief Return the current solver
     */
    QPSolverPtr getSolver(){return solver;}
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

#endif // WBC_CORE_SCENE_HPP
