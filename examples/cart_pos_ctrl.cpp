#include <wbc/WbcVelocity.hpp>
#include <wbc/robot_models/KinematicRobotModelKDL.hpp>
#include <wbc/solvers/HierarchicalLeastSquaresSolver.hpp>
#include <ctrl_lib/ProportionalFeedForwardController.hpp>
#include <wbc/robot_models/RobotModelConfig.hpp>
#include <wbc/common/ConstraintConfig.hpp>

using namespace wbc;
using namespace ctrl_lib;

int main(){

    WbcVelocity wbc;
    KinematicRobotModelKDL robot_model;
    HierarchicalLeastSquaresSolver solver;
    ProportionalFeedForwardController cart_pos_controller(6);
    ProportionalFeedForwardController jnt_pos_controller(1);

    ///// Create WBC config
    ///

    std::vector<ConstraintConfig> wbc_config;

    // Constraint for Cartesian Position Control
    ConstraintConfig cart_constraint;
    cart_constraint.name       = "cart_pos_ctrl";
    cart_constraint.type       = cart;
    cart_constraint.priority   = 0; // highest
    cart_constraint.root       = "kuka_lbr_base";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_base";
    cart_constraint.activation = 1;
    cart_constraint.weights    = std::vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    // Constraint for Joint Position Control
    ConstraintConfig jnt_constraint;
    jnt_constraint.name       = "jnt_pos_ctrl";
    jnt_constraint.type       = jnt;
    jnt_constraint.priority   = 1; // lower prio than cart constraint
    jnt_constraint.activation = 1;
    jnt_constraint.weights    = std::vector<double>(1,1);
    jnt_constraint.joint_names.push_back("kuka_lbr_l_joint_3");
    wbc_config.push_back(jnt_constraint);

    ///// Configure robot model
    ///

    RobotModelConfig robot_model_cfg("../data/kuka_lbr.urdf");
    if(!robot_model.loadModel(robot_model_cfg))  // Load urdf file
        return -1;
    if(!robot_model.addTaskFrames(wbc.getTaskFrameIDs(wbc_config)))  // add all task frame that are required
            return -1;

    std::cout<<"Configured robot model ..."<<std::endl;

    ///// Configure WBC
    ///

    if(!wbc.configure(wbc_config, robot_model.getJointNames()))
        return -1;

    std::cout<<"Configured wbc..."<<std::endl;

    ///// Configure Solver
    ///

    if(!solver.configure(wbc.getConstraintVariablesPerPrio(), robot_model.getJointNames().size()))
        return -1;

    std::cout<<"Configured solver..."<<std::endl;

    ///// Configure Controllers
    ///

    base::VectorXd prop_gain, max_ctrl_output;

    prop_gain.setConstant(6,1.0);
    max_ctrl_output.setConstant(6,0.1);
    cart_pos_controller.setPropGain(prop_gain);
    cart_pos_controller.setMaxControlOutput(max_ctrl_output);

    prop_gain.setConstant(1,1.0);
    max_ctrl_output.setConstant(1,0.1);
    jnt_pos_controller.setPropGain(prop_gain);
    jnt_pos_controller.setMaxControlOutput(max_ctrl_output);




    return 0;
}
