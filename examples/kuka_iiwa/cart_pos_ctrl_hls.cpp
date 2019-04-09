#include <robot_models/KinematicRobotModelKDL.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/WbcVelocityScene.hpp>
#include <solvers/hls/HierarchicalLSSolver.hpp>

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
    RobotModelPtr robot_model = make_shared<KinematicRobotModelKDL>();
    vector<RobotModelConfig> config(1);
    config[0].file = "../../../examples/kuka_iiwa/data/urdf/kuka_iiwa.urdf";
    if(!robot_model->configure(config, joint_names, "kuka_lbr_base"))
        return -1;

    wbc_solvers::HierarchicalLSSolver solver;
    std::vector<int> n_constraints_pp;
    n_constraints_pp.push_back(6);
    solver.configure(n_constraints_pp, robot_model->noOfJoints());

    // Configure WBC Scene
    WbcVelocityScene wbc_scene(robot_model);
    if(!wbc_scene.configure(wbc_config))
        return -1;

    // Set reference
    CartesianState target, ref, act;
    target.time = base::Time::now();
    target.pose.position = base::Vector3d(0.0, 0.0, 0.8);
    target.pose.orientation.setIdentity();
    act.pose.position.setZero();
    act.pose.orientation.setIdentity();

    // Run control loop
    base::samples::Joints joint_state;
    joint_state.resize(7);
    joint_state.names = robot_model->jointNames();
    for(int i = 0; i < 7; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now();

    base::VectorXd solver_output;

    double loop_time = 0.1; // seconds
    while((target.pose.position - act.pose.position).norm() > 1e-4){

        // Update robot model
        robot_model->update(joint_state);

        act = robot_model->cartesianState(cart_constraint.root, cart_constraint.tip);
        Twist diff;
        diff = target.pose - act.pose;
        ref.twist.linear = diff.linear;
        ref.twist.angular = diff.angular;
        ref.time = base::Time::now();
        shared_ptr<CartesianVelocityConstraint> constraint = static_pointer_cast<CartesianVelocityConstraint>(wbc_scene.getConstraint("cart_pos_ctrl_left"));
        constraint->setReference(ref);

        // Compute ctrl solution
        wbc_scene.update();
        HierarchicalQP qp;
        wbc_scene.getHierarchicalQP(qp);
        solver.solve(qp, solver_output);

        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += solver_output(i) * loop_time;

        cout<<"Target: x: "<<target.pose.position(0)<<" y: "<<target.pose.position(1)<<" z: "<<target.pose.position(2)<<endl;
        cout<<"Target: qx: "<<target.pose.orientation.x()<<" qy: "<<target.pose.orientation.y()<<" qz: "<<target.pose.orientation.z()<<" qw: "<<target.pose.orientation.w()<<endl<<endl;

        cout<<"Actual x: "<<act.pose.position(0)<<" y: "<<act.pose.position(1)<<" z: "<<act.pose.position(2)<<endl;
        cout<<"Actual qx: "<<act.pose.orientation.x()<<" qy: "<<act.pose.orientation.y()<<" qz: "<<act.pose.orientation.z()<<" qw: "<<act.pose.orientation.w()<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
