#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tasks/CartesianVelocityTask.hpp>
#include <unistd.h>

using namespace wbc;
using namespace std;
using namespace qpOASES;
using namespace wbc;

/**
 * Velocity-based example, Cartesian position control on 6 dof leg of the RH5 humanoid (fixed base/fully actuated, serial robot model). In the example the following problem is solved:
 *  \f[
 *        \begin{array}{ccc}
 *        minimize &  \| \mathbf{J}_w\dot{\mathbf{q}} - \mathbf{v}_d\|_2\\
 *        \mathbf{\dot{q}} & & \\
 *           s.t.  & \dot{\mathbf{q}}_m \leq \dot{\mathbf{q}} \leq \dot{\mathbf{q}}_M& \\
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
 * \f$\mathbf{x}_r, \mathbf{x}\f$ - Reference pose, actual pose<br>
 * \f$\mathbf{K}_p\f$ - Proportional feed forward gain matrix<br>
 * \f$\mathbf{\dot{q}}_m,\mathbf{\dot{q}}_M\f$ - Joint velocity limits<br>
 *
 * The robot ankle is supposed to move to a fixed target pose. The QP is solved using the QPOases solver. Note that the robot has a fixed base here and there are no rigid contacts that have to be considered.
 * In the example, the Hyrodyn robot model is used, however, with a serial URDF model, i.e., the output velocities are in independent joint space.
 */
int main(){

    double dt = 0.001;

    // Create Hyrodyn based robot model.  In this case, we use the Hyrodyn-based model, which allows handling of parallel mechanisms.
    RobotModelPtr robot_model = make_shared<RobotModelPinocchio>();

    // Configure the model. We pass a serial model description to RobotModelHyrodyn, which means that the solution will be identical
    // to the one obtained by  RobotModelPinocchio. RobotModelHyrodyn assumes that you specify correctly all joints (actuated and unactuated).
    // Also, we have to pass a submechanism files, which describes the parallel structures inside the robot architecture
    // In this case, there are not parallel structure, since we use the abstract model
    RobotModelConfig config;
    config.file_or_string = "../../../models/rh5/urdf/rh5_single_leg.urdf";
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
    if(!robot_model->configure(config))
        return -1;

    // Independent joint names. Since the model is serial and we don' t have a floating base:
    // actuated joints == independend joints == all joints
    vector<string> ind_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"};

    // Configure solver, use QPOASES in this case
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene, use VelocitySceneQuadraticCost in this case
    CartesianVelocityTaskPtr cart_task;
    cart_task = make_shared<CartesianVelocityTask>(TaskConfig("left_leg_posture",0,{1,1,1,1,1,1},1),
                                                   "LLAnkle_FT",
                                                   "RH5_Root_Link",
                                                   robot_model->nj());
    VelocitySceneQP scene(robot_model, solver, dt);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    Eigen::VectorXd p_gain(6);
    p_gain << 10.0,10.0,10.0,10.0,10.0,10.0;
    controller.setPGain(p_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed
    uint nj = ind_joint_names.size();
    types::JointState joint_state;
    joint_state.resize(6);
    joint_state.position << 0,0,-0.2,0.4,0,-0.2;

    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::Pose ref_pose, pose;
    types::Twist ref_twist, ctrl_output;
    ref_pose.position = Eigen::Vector3d(0, 0, -0.7);
    ref_pose.orientation = Eigen::Quaterniond(0,-1,0,0);
    pose.position.setZero();
    pose.orientation.setIdentity();

    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    while((ref_pose.position - pose.position).norm() > 1e-3){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        pose = robot_model->pose(cart_task->tipFrame());
        ctrl_output = controller.update(ref_pose,ref_twist,pose);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Update the current joint state. Simply integrate the current joint position using the joint velocity given by the solver.
        // On a real robot, this would be replaced by a function that sends the solver output to the joints.
        joint_state.position += solver_output.velocity * loop_time;

        cout<<"setpoint: x:   "<<ref_pose.position(0)<<" y: "<<ref_pose.position(1)<<" z: "<<ref_pose.position(2)<<endl;
        cout<<"setpoint: qx:  "<<ref_pose.orientation.x()<<" qy: "<<ref_pose.orientation.y()<<" qz: "<<ref_pose.orientation.z()<<" qw: "<<ref_pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:    "<<pose.position(0)<<" y: "<<pose.position(1)<<" z: "<<pose.position(2)<<endl;
        cout<<"feedback qx:   "<<pose.orientation.x()<<" qy: "<<pose.orientation.y()<<" qz: "<<pose.orientation.z()<<" qw: "<<pose.orientation.w()<<endl<<endl;
        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(uint i = 0; i < nj; i++) cout<<robot_model->jointNames()[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; cout<<solver_output.velocity.transpose()<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
