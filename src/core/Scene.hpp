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
    Eigen::VectorXd solver_output;
    double hessian_regularizer;
    std::vector<types::Wrench> contact_wrenches;

public:
    Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double dt);
    ~Scene();

    /**
     * @brief Configure the WBC scene. Create tasks and sort them by priority given the task config
     * @param tasks Tasks used in optimization function. Size has to be > 0. All tasks have to be valid. See tasks and TaskConfig.hpp for more details.
     */
    virtual bool configure(const std::vector<TaskPtr> &tasks) = 0;

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
     * @brief Return the current robot model
     */
    RobotModelPtr getRobotModel(){return robot_model;}

    /**
     * @brief Return the current solver
     */
    QPSolverPtr getSolver(){return solver;}

    /**
     * @brief Get current solver output in raw values
     */
    const Eigen::VectorXd& getSolverOutputRaw() const { return solver_output; }

    /**
     * @brief setHessianRegularizer
     * @param reg This value is added to the diagonal of the Hessian matrix inside the QP to reduce the risk of infeasibility. Default is 1e-8
     */
    void setHessianRegularizer(const double reg){hessian_regularizer=reg;}

    /**
     * @brief Return the current value of hessian regularizer
     */
    double getHessianRegularizer(){return hessian_regularizer;}

    /**
     * @brief Get estimated contact wrenches
     */
    const std::vector<types::Wrench>& getContactWrenches(){return contact_wrenches;}
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
