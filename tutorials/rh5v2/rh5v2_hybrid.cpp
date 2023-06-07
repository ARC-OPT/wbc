#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/acceleration_tsid/AccelerationSceneTSID.hpp>
#include <tools/JointIntegrator.hpp>
#include <controllers/CartesianPosPDController.hpp>

using namespace std;
using namespace wbc;
using namespace ctrl_lib;

/**
 * Acceleration-based example, same problem as in the rh5v2 example. The only difference is that here we use the hybrid robot model, i.e. we
 * consider the parallel structures in the mechanical structure and solve the WBC problem in actuation space. The output is thus not an acceleration/torque vector
 * of the independent joints, but the acceleration andforces/torques of the actual actuators. In case of the RH5v2 arms, there are two such parallel structures:
 *
 * - The elbow joint is driven by a linear actuator
 * - The wrist structure is a parallel mechanism with two linear actuators
 */
int main()
{
    double dt = 0.01;

    // Create robot model, use Hyrodyn based model
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();

    // Configure robot model. Use the hybrid model this time.
    RobotModelConfig config("../../../models/rh5v2/urdf/rh5v2_hybrid.urdf");
    config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
    if(!robot_model->configure(config))
        return -1;

    hyrodyn::RobotModel_HyRoDyn* rm_hyrodyn = std::dynamic_pointer_cast<RobotModelHyrodyn>(robot_model)->hyrodynHandle();

    // Configure solver, use QPOases
    QPSolverPtr solver = make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    // Configure the AccelerationSceneTSID scene. This scene computes joint accelerations, joint torques and contact wrenches as output.
    // Pass two tasks here: Left arm Cartesian pose and right arm Cartesian pose.
    AccelerationSceneTSID scene(robot_model, solver, dt);
    vector<TaskConfig> wbc_config;
    wbc_config.push_back(TaskConfig("cart_ctrl_left",  0, "RH5v2_Root_Link", "ALWristFT_Link", "RH5v2_Root_Link", 1.0));
    wbc_config.push_back(TaskConfig("cart_ctrl_right",  0, "RH5v2_Root_Link", "ARWristFT_Link", "RH5v2_Root_Link", 1.0));
    if(!scene.configure(wbc_config))
        return -1;

    // Choose an initial joint state. Since we use acceleration-based WBC here, we have to pass the velocities and
    // accelerations as well
    base::samples::Joints joint_state;
    joint_state.names = rm_hyrodyn->jointnames_independent;
    uint nj = joint_state.names.size();
    for(auto n : joint_state.names){
        base::JointState js;
        js.position = js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }
    joint_state.time = base::Time::now();
    joint_state["ALShoulder1"].position = joint_state["ARShoulder1"].position = -1.0;
    joint_state["ALShoulder2"].position = joint_state["ARShoulder2"].position = 1.0;
    joint_state["ALElbow"].position     = joint_state["ARElbow"].position     = -1.0;
    robot_model->update(joint_state);

    // Configure Cartesian controllers. The controllers implement the following control law:
    //
    //    a_d = kf*a_r +  kd*(v_r - v) + kp(x_r - x).
    //
    // As we don't use feed forward acceleration here, we can ignore the factor kf.
    CartesianPosPDController ctrl_left, ctrl_right;
    base::VectorXd p_gain(6),d_gain(6),ff_gain(6);
    p_gain.setConstant(10); // Stiffness
    d_gain.setConstant(30); // Damping
    ff_gain.setConstant(1); // Feed forward
    ctrl_left.setPGain(p_gain);
    ctrl_left.setDGain(d_gain);
    ctrl_right.setPGain(p_gain);
    ctrl_right.setDGain(d_gain);

    // Target Pose left/right
    base::samples::RigidBodyStateSE3 setpoint_left, setpoint_right, feedback_left, feedback_right, ctrl_output_left, ctrl_output_right;
    setpoint_left.pose.position = base::Vector3d(0.4,0.453543,0.183343);
    setpoint_left.pose.orientation = base::Quaterniond(0.371912,-0.485673,0.725747,-0.314793);
    setpoint_right.pose.position = base::Vector3d(0.522827, -0.453543, 0.183345);
    setpoint_right.pose.orientation = base::Quaterniond(0.371914, 0.485672, 0.72575, 0.314787);

    // Run control loop
    JointIntegrator integrator;
    double loop_time = dt; // seconds
    base::commands::Joints solver_output;
    for(double t = 0; t < 5; t+=loop_time){

        // Update the robot model. WBC will only work if at least one joint state with valid timestamp has been passed to the robot model.
        robot_model->update(joint_state);

        // Update controllers, left arm: Follow sinusoidal trajectory
        setpoint_left.pose.position[0] = 0.522827 + 0.1*sin(t);
        setpoint_left.twist.linear[0] = 0.1*cos(t);
        setpoint_left.acceleration.linear[0] = -0.1*sin(t);
        feedback_left = robot_model->rigidBodyState(wbc_config[0].root, wbc_config[0].tip);
        feedback_right = robot_model->rigidBodyState(wbc_config[1].root, wbc_config[1].tip);
        ctrl_output_left = ctrl_left.update(setpoint_left, feedback_left);
        ctrl_output_right = ctrl_right.update(setpoint_right, feedback_right);

        // Update constraints. Pass the control output of the controller to the corresponding constraint.
        // The control output is the gradient of the task function that is to be minimized during execution.
        scene.setReference(wbc_config[0].name, ctrl_output_left);
        scene.setReference(wbc_config[1].name, ctrl_output_right);

        // Update WBC scene. The output is a (hierarchical) quadratic program (QP), which can be solved by any standard QP solver
        HierarchicalQP hqp = scene.update();

        // Solve the QP. The output is the joint acceleration/torque that achieves the task space acceleration demanded by the controllers
        solver_output = scene.solve(hqp);

        // Integrate once to get joint velocity from solver output
        integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()),solver_output,loop_time);

        // Use Hyrodyn to compute the joint velocity in independent joint space from the solver output, which only contat
        for(int i = 0; i < solver_output.size(); i++)
            rm_hyrodyn->ud[i] = solver_output[i].speed;
        rm_hyrodyn->calculate_forward_system_state();

        // Update joint state by integrating again
        for(size_t i = 0; i < joint_state.size(); i++){
            joint_state[i].position += rm_hyrodyn->yd[i] * loop_time;
            joint_state[i].speed = rm_hyrodyn->yd[i];
        }

        cout<<"setpoint left: x: "<<setpoint_left.pose.position(0)<<" y: "<<setpoint_left.pose.position(1)<<" z: "<<setpoint_left.pose.position(2)<<endl;
        cout<<"setpoint left: qx: "<<setpoint_left.pose.orientation.x()<<" qy: "<<setpoint_left.pose.orientation.y()<<" qz: "<<setpoint_left.pose.orientation.z()<<" qw: "<<setpoint_left.pose.orientation.w()<<endl<<endl;
        cout<<"feedback left x: "<<feedback_left.pose.position(0)<<" y: "<<feedback_left.pose.position(1)<<" z: "<<feedback_left.pose.position(2)<<endl;
        cout<<"feedback left qx: "<<feedback_left.pose.orientation.x()<<" qy: "<<feedback_left.pose.orientation.y()<<" qz: "<<feedback_left.pose.orientation.z()<<" qw: "<<feedback_left.pose.orientation.w()<<endl<<endl;

        cout<<"setpoint right: x: "<<setpoint_right.pose.position(0)<<" y: "<<setpoint_right.pose.position(1)<<" z: "<<setpoint_right.pose.position(2)<<endl;
        cout<<"setpoint right: qx: "<<setpoint_right.pose.orientation.x()<<" qy: "<<setpoint_right.pose.orientation.y()<<" qz: "<<setpoint_right.pose.orientation.z()<<" qw: "<<setpoint_right.pose.orientation.w()<<endl<<endl;
        cout<<"feedback right x: "<<feedback_right.pose.position(0)<<" y: "<<feedback_right.pose.position(1)<<" z: "<<feedback_right.pose.position(2)<<endl;
        cout<<"feedback right qx: "<<feedback_right.pose.orientation.x()<<" qy: "<<feedback_right.pose.orientation.y()<<" qz: "<<feedback_right.pose.orientation.z()<<" qw: "<<feedback_right.pose.orientation.w()<<endl<<endl;

        cout<<"Solver output: "; cout<<endl;
        cout<<"Joint Names:   "; for(int i = 0; i < nj; i++) cout<<solver_output.names[i]<<" "; cout<<endl;
        cout<<"Velocity:      "; for(int i = 0; i < nj; i++) cout<<solver_output[i].speed<<" "; cout<<endl;
        cout<<"Acceleration:  "; for(int i = 0; i < nj; i++) cout<<solver_output[i].acceleration<<" "; cout<<endl;
        cout<<"Torque:        "; for(int i = 0; i < nj; i++) cout<<solver_output[i].effort<<" "; cout<<endl;
        cout<<"---------------------------------------------------------------------------------------------"<<endl<<endl;
        usleep(loop_time * 1e6);
    }

    return 0;
}
