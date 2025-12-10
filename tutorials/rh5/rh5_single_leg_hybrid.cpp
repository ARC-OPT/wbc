#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/pinocchio/RobotModelPinocchio.hpp>
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <core/RobotModelConfig.hpp>
#include <scenes/velocity_qp/VelocitySceneQP.hpp>
#include <tasks/SpatialVelocityTask.hpp>
#include <controllers/CartesianPosPDController.hpp>
#include <tools/JointIntegrator.hpp>

using namespace wbc;
using namespace std;
using namespace qpOASES;

/**
 * Velocity-based example, exact same problem as in the rh5_single_leg example. The only difference is that here we use the hybrid robot model, i.e. we
 * consider the parallel structures in the mechanical structure and solve the WBC problem in actuation space. The output is thus not a velocity vector
 * of the independent joints, but the velocities of the actual actuators. In case of the RH5 leg, there are three such parallel structures:
 *
 * - The Hip3 joint is driven by a linear actuator
 * - The Knee joint is driven by a linear actuator
 * - The ankle structure is a parallel mechanism with two linear actuators
 */
int main(){

    double dt = 0.001;

    // Create Hyrodyn based robot model.  In this case, we use the Hyrodyn-based model, which allows handling of parallel mechanisms.
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();

    // Configure the full robot model, containing all linear actuators and parallel mechanisms.
    RobotModelConfig config;
    config.file_or_string = "../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf";
    config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
    if(!robot_model->configure(config))
        return -1;

    // Independent joint names. This time they differ from the controlled joints in WBC
    vector<string> ind_joint_names = {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"};

    // Configure solver, use QPOASES in this case
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    // Configure Scene, use VelocitySceneQuadraticCost in this case
    SpatialVelocityTaskPtr cart_task = std::make_shared<SpatialVelocityTask>(TaskConfig("left_leg_posture",0,Eigen::VectorXd::Ones(6),1),
                                                                                        robot_model,
                                                                                        "LLAnkle_FT");
   VelocitySceneQP scene(robot_model, solver, dt);
    if(!scene.configure({cart_task}))
        return -1;

    // Configure the controller. In this case, we use a Cartesian position controller. The controller implements the following control law:
    //
    //    v_d = kd*v_r + kp(x_r - x).
    //
    // As we don't use feed forward velocity here, we can ignore the factor kd.
    CartesianPosPDController controller;
    Eigen::VectorXd p_gain(6),d_gain(6);
    p_gain.setConstant(10); // Stiffness
    d_gain.setZero();  // No damping
    controller.setPGain(p_gain);  
    controller.setDGain(d_gain);

    // Choose an initial joint state. For velocity-based WBC only the current position of all joint has to be passed.
    // Note that WBC takes as input ALWAYS the independent joints, no matter what robot model is used.
    uint nj = ind_joint_names.size();
    types::JointState joint_state;
    joint_state.resize(robot_model->na());
    joint_state.position << 0,0,-0.2,0.4,0,-0.2;
    joint_state.velocity.setZero();
    joint_state.acceleration.setZero();
    
    // Choose a valid reference pose x_r, which is defined in cart_task.ref_frame and defines the desired pose of
    // the cart_task.ref_tip frame. The pose will be passed as setpoint to the controller.
    types::RigidBodyState setpoint, feedback;
    types::Twist ctrl_output;
    setpoint.pose.position = Eigen::Vector3d(0,0, -0.7);
    setpoint.pose.orientation = Eigen::Quaterniond(0,-1,0,0);
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    cout<<robot_model->nj()<<endl;

    // Run control loop
    double loop_time = dt; // seconds
    types::JointCommand solver_output;
    JointIntegrator joint_integrator;
    while((setpoint.pose.position - feedback.pose.position).norm() > 1e-3){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model
        robot_model->update(joint_state.position,
                            joint_state.velocity,
                            joint_state.acceleration);

        // Update controller. The feedback is the pose of the tip link described in ref_frame link
        feedback.pose = robot_model->pose(cart_task->tipFrame());
        ctrl_output = controller.update(setpoint.pose, setpoint.twist, feedback.pose);

        // Update constraints. Pass the control output of the solver to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        cart_task->setReference(ctrl_output);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint velocity that achieves the task space velocity demanded by the controller, i.e.,
        // this joint velocity will drive the end effector to the reference x_r
        solver_output = scene.solve(hqp);

        // Use Hyrodyn to compute the joint velocity in independent joint space from the solver output, which contains only the actuated joints.
        // Normally, we would send the solver output directly to the actuators of our robot
        hyrodyn::RobotModel_HyRoDyn* rm_hyrodyn = std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->hyrodynHandle();
        rm_hyrodyn->ud = solver_output.velocity;
        rm_hyrodyn->calculate_forward_system_state();

        joint_state.velocity     = rm_hyrodyn->yd.tail(robot_model->na());
        joint_state.position     += joint_state.velocity * dt;

        // Update independent joint state

        cout<<"setpoint: x:    "<<setpoint.pose.position(0)<<" y: "<<setpoint.pose.position(1)<<" z: "<<setpoint.pose.position(2)<<endl;
        cout<<"setpoint: qx:   "<<setpoint.pose.orientation.x()<<" qy: "<<setpoint.pose.orientation.y()<<" qz: "<<setpoint.pose.orientation.z()<<" qw: "<<setpoint.pose.orientation.w()<<endl<<endl;
        cout<<"feedback x:     "<<feedback.pose.position(0)<<" y: "<<feedback.pose.position(1)<<" z: "<<feedback.pose.position(2)<<endl;
        cout<<"feedback qx:    "<<feedback.pose.orientation.x()<<" qy: "<<feedback.pose.orientation.y()<<" qz: "<<feedback.pose.orientation.z()<<" qw: "<<feedback.pose.orientation.w()<<endl<<endl;
        cout<<"Solver output:  "; cout<<endl;
        cout<<"Velocity:       "<<solver_output.velocity.transpose()<<" "<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;

        usleep(loop_time * 1e6);
    }

    return 0;
}
