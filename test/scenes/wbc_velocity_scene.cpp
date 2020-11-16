#include <boost/test/unit_test.hpp>
#include "robot_models/RobotModelKDL.hpp"
#include "core/RobotModelConfig.hpp"
#include "scenes/WbcVelocityScene.hpp"

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(test_configure){

    vector<string> joint_names;
    for(int i = 0; i < 7; i++)
        joint_names.push_back("kuka_lbr_l_joint_" + to_string(i+1));

    // Create WBC config
    vector<ConstraintConfig> wbc_config;

    // Constraint for Cartesian Position Control
    ConstraintConfig cart_constraint;
    cart_constraint.name       = "cart_pos_ctrl_left";
    cart_constraint.type       = cart;
    cart_constraint.priority   = 0;
    cart_constraint.root       = "kuka_lbr_base";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_base";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    // Configure Robot model
    shared_ptr<RobotModelKDL> robot_model = make_shared<RobotModelKDL>();
    vector<RobotModelConfig> config(1);
    config[0].file = std::string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/control/wbc/test/data/kuka_lbr.urdf";
    config[0].joint_names = joint_names;
    BOOST_CHECK_EQUAL(robot_model->configure(config), true);

    // Configure WBC Scene
    WbcVelocityScene wbc_scene(robot_model);
    BOOST_CHECK_EQUAL(wbc_scene.configure(wbc_config), true);
}
