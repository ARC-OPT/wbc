#include <robot_models/RobotModelKDL.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/VelocityScene.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;

int main(int argc, char** argv){

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
    cart_constraint.root       = "kuka_lbr_l_link_0";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_l_link_0";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    wbc_config.push_back(cart_constraint);

    // Configure Robot model
    RobotModelPtr robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../examples/kuka_iiwa/data/urdf/kuka_iiwa.urdf";
    config.joint_names = joint_names;
    config.actuated_joint_names = joint_names;
    if(!robot_model->configure(config))
        return -1;

    // Create solver
    QPOASESSolver solver;
    qpOASES::Options options;
    options.setToDefault();
    options.printLevel = qpOASES::PL_NONE;
    solver.setMaxNoWSR(100);
    solver.setOptions(options);

    // Configure WBC Scene
    VelocityScene wbc_scene(robot_model);
    if(!wbc_scene.configure(wbc_config))
        return -1;

    // Create controller
    ctrl_lib::CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1));

    // Set reference
    base::samples::RigidBodyStateSE3 setpoint, ctrl_output, feedback;
    setpoint.time = base::Time::now();
    setpoint.pose.position = base::Vector3d(0.0, 0.0, 0.8);
    setpoint.pose.orientation.setIdentity();
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    base::samples::Joints joint_state;
    joint_state.resize(7);
    joint_state.names = robot_model->jointNames();
    for(int i = 0; i < 7; i++)
        joint_state[i].position = 0.01;
    joint_state.time = base::Time::now();

    base::VectorXd solver_output;

    double loop_time = 0.1; // seconds
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){

        // Update robot model
        robot_model->update(joint_state);

        // Update controllers
        feedback = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints
        shared_ptr<CartesianVelocityConstraint> constraint = static_pointer_cast<CartesianVelocityConstraint>(wbc_scene.getConstraint("cart_pos_ctrl_left"));
        constraint->setReference(ctrl_output);

        // Update WBC scene and solve
        wbc_scene.update();
        HierarchicalQP qp;
        wbc_scene.getHierarchicalQP(qp);
        solver.solve(qp, solver_output);

        // Update joint state
        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += solver_output(i) * loop_time;
        joint_state.time = base::Time::now();

        cout<<"setpoint: x: "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx: "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;

        cout<<"feedback x: "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx: "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;


        cout<<"Solver output: "; for(int i = 0; i < 7; i++) cout<<solver_output(i)<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
