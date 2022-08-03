#include <boost/test/unit_test.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include "tools/URDFTools.hpp"
#include "robot_models/rbdl/RobotModelRBDL.hpp"

using namespace std;
using namespace wbc;
using namespace RigidBodyDynamics;

BOOST_AUTO_TEST_CASE(configuration){

    RobotModelRBDL robot_model;
    RobotModelConfig cfg("../../../../models/rh5/urdf/rh5.urdf");
    BOOST_CHECK(robot_model.configure(cfg));


    cfg.floating_base = true;
    BOOST_CHECK(robot_model.configure(cfg));
}
