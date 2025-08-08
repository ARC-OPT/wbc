#include "Scene.hpp"
#include "Task.hpp"
#include "../tools/Logger.hpp"

loglevel_e loglevel = logWARNING;

namespace wbc{

Scene::Scene(RobotModelPtr robot_model, QPSolverPtr solver, const double /*dt*/) :
    robot_model(robot_model),
    solver(solver),
    hessian_regularizer(1e-8){
}

Scene::~Scene(){
}

SceneFactory::SceneMap* SceneFactory::scene_map = 0;


} // namespace wbc
