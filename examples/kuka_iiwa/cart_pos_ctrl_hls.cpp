#include <robot_models/kdl/RobotModelKDL.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/VelocityScene.hpp>
#include <solvers/hls/HierarchicalLSSolver.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <unistd.h>

using namespace std;
using namespace wbc;

/**
 * Simple Velocity-based example, Cartesian position control on a kuka iiwa 7 dof arm. The solution is computed using the hierarchical least
 * squares solver (hls), which provides an analytical solution. In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize & \| \dot{\mathbf{q}} \|_2& \\
 *            \mathbf{\dot{q}} & & \\
 *           s.t. & \mathbf{J}_w\dot{\mathbf{q}}=\mathbf{v}_{d}
 *        \end{array}
 *  \f]
 * where
 *  \f[
 *        \mathbf{v}_d = \mathbf{K}_p(\mathbf{x}_r-\mathbf{x})
 *  \f]
 * \f$\dot{\mathbf{q}}\f$ - Vector of robot joint velocities<br>
 * \f$\mathbf{v}_{d}\f$ - Desired spatial velocity / controller output <br>
 * \f$\mathbf{J}_w=\mathbf{W}\mathbf{J}\f$ - Weighted task Jacobian <br>
 * \f$\mathbf{W}\f$ - Diagonal weight matrix<br>
 * \f$\mathbf{X}_r,\mathbf{X}\f$ - Reference pose, actual pose<br>
 * \f$\mathbf{K}_p\f$ - Proportional gain matrix<br>
 *
 * The robot end effector is supposed to move to a fixed target pose. As there is only one task, the soluton degrades to a simple pseudo inversion / linear least squares.
 * The task weights are all set to 1 initially, i.e., all task variables are treated with equal priority. The solution resolves the redundancy by minimizing the kinetic energy,
 * i.e., it provides the minimal joint velocities that produce the desired spatial velocity.
 */
int main(int argc, char** argv){

    // Configure Robot model
    RobotModelPtr robot_model = make_shared<RobotModelKDL>();
    RobotModelConfig config;
    config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
    if(!robot_model->configure(config))
        return -1;

    // Configure solver
    QPSolverPtr solver = std::make_shared<HierarchicalLSSolver>();
    std::dynamic_pointer_cast<HierarchicalLSSolver>(solver)->setMaxSolverOutputNorm(10); // Maximum allowed norm of the solution vector

    // Configure WBC Scene
    ConstraintConfig cart_constraint;
    cart_constraint.name       = "cart_pos_ctrl_left";
    cart_constraint.type       = cart;
    cart_constraint.priority   = 0;
    cart_constraint.root       = "kuka_lbr_l_link_0";
    cart_constraint.tip        = "kuka_lbr_l_tcp";
    cart_constraint.ref_frame  = "kuka_lbr_l_link_0";
    cart_constraint.activation = 1;
    cart_constraint.weights    = vector<double>(6,1);
    VelocityScene scene(robot_model, solver);
    if(!scene.configure({cart_constraint}))
        return -1;

    // Configure controller
    ctrl_lib::CartesianPosPDController controller;
    controller.setPGain(base::Vector6d::Constant(1));

    // Initial joint state
    base::samples::Joints joint_state;
    uint nj = robot_model->noOfJoints();
    joint_state.resize(nj);
    joint_state.names = robot_model->jointNames();
    for(int i = 0; i < nj; i++)
        joint_state[i].position = 0.1;
    joint_state.time = base::Time::now();

    // Reference pose
    base::samples::RigidBodyStateSE3 setpoint, ctrl_output, feedback;
    setpoint.pose.position = base::Vector3d(0.0, 0.0, 0.8);
    setpoint.pose.orientation.setIdentity();
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    // Run control loop
    double loop_time = 0.01; // seconds
    base::commands::Joints solver_output;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-4){

        // Update robot model
        robot_model->update(joint_state);

        // Update controllers
        feedback = robot_model->rigidBodyState(cart_constraint.root, cart_constraint.tip);
        ctrl_output = controller.update(setpoint, feedback);

        // Update constraints
        scene.setReference(cart_constraint.name, ctrl_output);

        // Update WBC scene and solve
        HierarchicalQP hqp = scene.update();
        solver_output = scene.solve(hqp);

        // Update joint state
        for(size_t i = 0; i < joint_state.size(); i++)
            joint_state[i].position += solver_output[i].speed * loop_time;
        joint_state.time = base::Time::now();

        cout<<"setpoint: x:   "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx:  "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:    "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx:   "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
