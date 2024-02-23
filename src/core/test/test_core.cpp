#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <core/TaskConfig.hpp>
#include <core/PluginLoader.hpp>
#include <core/RobotModel.hpp>
#include <core/QPSolver.hpp>
#include <core/Scene.hpp>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(task_config){
    // Cartesian task config
    TaskConfig cart_config;
    cart_config.name       = "cart_pos_ctrl_left";
    cart_config.type       = cart;
    cart_config.priority   = 0;
    cart_config.root       = "kuka_lbr_l_link_0";
    cart_config.tip        = "kuka_lbr_l_tcp";
    cart_config.ref_frame  = "kuka_lbr_l_link_0";
    cart_config.activation = 1;
    cart_config.weights    = vector<double>(6,1);

    // Valid cart_config
    BOOST_CHECK_NO_THROW(cart_config.validate());

    // Invalid task type
    cart_config.type = unset;
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.type = cart;

    // Invalid priority
    cart_config.priority = -1;
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.priority = 0;

    // Invalid activation
    cart_config.activation = -1;
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.activation = 1;

    // Invalid weight vector
    cart_config.weights = vector<double>(5,1);
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.weights = vector<double>(6,1);
    cart_config.weights[1] = -1;
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.weights[1] = 1;

    // Empty frame ID
    cart_config.root = "";
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.root = "kuka_lbr_l_tcp";
    cart_config.ref_frame = "";
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.ref_frame = "kuka_lbr_l_link_0";

    cart_config.tip = "";
    BOOST_CHECK_THROW(cart_config.validate(), std::invalid_argument);
    cart_config.tip = "kuka_lbr_l_link_0";

    // Joint task config
    TaskConfig jnt_config;
    jnt_config.name        = "jnt_pos_ctrl";
    jnt_config.type        = jnt;
    jnt_config.priority    = 0;
    jnt_config.joint_names = {"joint_1", "joint_2", "joint_3"};
    jnt_config.activation  = 1;
    jnt_config.weights     = vector<double>(3,1);

    // Valid config
    BOOST_CHECK_NO_THROW(jnt_config.validate());

    // Invlaid number of weights
    jnt_config.weights = vector<double>(2,1);
    BOOST_CHECK_THROW(jnt_config.validate(), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(robot_model_factory){
    BOOST_CHECK_NO_THROW(PluginLoader::loadPlugin("libwbc-robot_models-pinocchio.so"));
    PluginLoader::PluginMap *plugin_map = PluginLoader::getPluginMap();
    BOOST_CHECK(plugin_map->count("libwbc-robot_models-pinocchio.so") == 1);
    RobotModelFactory::RobotModelMap *robot_model_map = RobotModelFactory::getRobotModelMap();
    BOOST_CHECK(robot_model_map->count("pinocchio") == 1);
    RobotModel* model;
    BOOST_CHECK_NO_THROW(model = RobotModelFactory::createInstance("pinocchio"));
    BOOST_CHECK(model != 0);
    BOOST_CHECK_NO_THROW(PluginLoader::unloadPlugin("libwbc-robot_models-pinocchio.so"));
}

BOOST_AUTO_TEST_CASE(qp_solver_factory){
    BOOST_CHECK_NO_THROW(PluginLoader::loadPlugin("libwbc-solvers-qpoases.so"));
    QPSolverFactory::QPSolverMap *qp_solver_map = QPSolverFactory::getQPSolverMap();
    BOOST_CHECK(qp_solver_map->size() == 1);
    BOOST_CHECK(qp_solver_map->count("qpoases") == 1);
    BOOST_CHECK(qp_solver_map->at("qpoases") != 0);
    QPSolver* model;
    BOOST_CHECK_NO_THROW(model = QPSolverFactory::createInstance("qpoases"));
    BOOST_CHECK(model != 0);
}

BOOST_AUTO_TEST_CASE(scene_factory){
    BOOST_CHECK_NO_THROW(PluginLoader::loadPlugin("libwbc-scenes-velocity.so"));
    SceneFactory::SceneMap *scene_map = SceneFactory::getSceneMap();
    BOOST_CHECK(scene_map->size() == 1);
    BOOST_CHECK(scene_map->count("velocity") == 1);
    BOOST_CHECK(scene_map->at("velocity") != 0);
    Scene* scene;
    RobotModelPtr robot_model;
    QPSolverPtr solver;
    BOOST_CHECK_NO_THROW(scene = SceneFactory::createInstance("velocity", robot_model, solver, 1e-3));
    BOOST_CHECK(scene != 0);
}
