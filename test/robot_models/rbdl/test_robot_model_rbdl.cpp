#include <boost/test/unit_test.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include "tools/URDFTools.hpp"
#include "robot_models/rbdl/RobotModelRBDL.hpp"

using namespace std;
using namespace wbc;
using namespace RigidBodyDynamics;

void printRbs(base::samples::RigidBodyStateSE3 rbs){
    cout<<"Position:      "<<rbs.pose.position.transpose()<<endl;
    cout<<"Orientation:   "<<rbs.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist linear:  "<<rbs.twist.linear.transpose()<<endl;
    cout<<"Twist angular: "<<rbs.twist.angular.transpose()<<endl;
    cout<<"Acc linear:    "<<rbs.acceleration.linear.transpose()<<endl;
    cout<<"Acc angular:   "<<rbs.acceleration.angular.transpose()<<endl<<endl;
}

base::samples::Joints makeRandomJointState(vector<string> joint_names){
    base::samples::Joints joint_state;
    joint_state.names = joint_names;
    for(uint i = 0; i < joint_names.size(); i++){
        base::JointState state;
        state.position = (double)rand()/RAND_MAX;
        state.speed = (double)rand()/RAND_MAX;
        state.acceleration = (double)rand()/RAND_MAX;
        joint_state.elements.push_back(state);
    }
    joint_state.time = base::Time::now();
    return joint_state;
}

base::samples::RigidBodyStateSE3 makeRandomFloatingBaseState(){
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear  = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.linear  = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.twist.angular = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    return floating_base_state;
}

BOOST_AUTO_TEST_CASE(configure_and_update){

    srand(time(NULL));

    vector<string> joint_names = {"BodyPitch","BodyRoll","BodyYaw",
                                  "ALShoulder1","ALShoulder2","ALShoulder3","ALElbow","ALWristRoll","ALWristYaw","ALWristPitch",
                                  "ARShoulder1","ARShoulder2","ARShoulder3","ARElbow","ARWristRoll","ARWristYaw","ARWristPitch",
                                  "HeadPitch","HeadRoll","HeadYaw",
                                  "LLHip1","LLHip2","LLHip3","LLKnee","LLAnkleRoll","LLAnklePitch",
                                  "LRHip1","LRHip2","LRHip3","LRKnee","LRAnkleRoll","LRAnklePitch"};

    const int nj = joint_names.size();

    // Valid config
    RobotModelRBDL robot_model;
    RobotModelConfig cfg("../../../../models/rh5/urdf/rh5.urdf");
    BOOST_CHECK(robot_model.configure(cfg));
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.independentJointNames()[i]);
        BOOST_CHECK(i == robot_model.jointIndex(joint_names[i]));
    }

    base::samples::Joints joint_state_in = makeRandomJointState(joint_names);
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in));
    base::samples::Joints joint_state_out = robot_model.jointState(joint_names);
    for(auto n : joint_names){
        BOOST_CHECK(joint_state_out[n].position = joint_state_in[n].position);
        BOOST_CHECK(joint_state_out[n].speed = joint_state_in[n].speed);
        BOOST_CHECK(joint_state_out[n].acceleration = joint_state_in[n].acceleration);
    }

    // Valid config with floating base
    cfg.floating_base = true;
    BOOST_CHECK(robot_model.configure(cfg));
    for(size_t i = 0; i < robot_model.noOfJoints(); i++){
        BOOST_CHECK(joint_names[i] == robot_model.jointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.actuatedJointNames()[i]);
        BOOST_CHECK(joint_names[i] == robot_model.independentJointNames()[i]);
        BOOST_CHECK(i == robot_model.jointIndex(joint_names[i]));
    }

    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 floating_base_state_out = robot_model.floatingBaseState();
    joint_state_out = robot_model.jointState(joint_names);
    for(auto n : joint_names){
        BOOST_CHECK(joint_state_out[n].position = joint_state_in[n].position);
        BOOST_CHECK(joint_state_out[n].speed = joint_state_in[n].speed);
        BOOST_CHECK(joint_state_out[n].acceleration = joint_state_in[n].acceleration);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(floating_base_state_in.pose.position[i] == floating_base_state_out.pose.position[i]);
        BOOST_CHECK(floating_base_state_in.pose.orientation.coeffs()[i] == floating_base_state_out.pose.orientation.coeffs()[i]);
        BOOST_CHECK(floating_base_state_in.twist.linear[i] == floating_base_state_out.twist.linear[i]);
        BOOST_CHECK(floating_base_state_in.twist.angular[i] == floating_base_state_out.twist.angular[i]);
        BOOST_CHECK(floating_base_state_in.acceleration.linear[i] == floating_base_state_out.acceleration.linear[i]);
        BOOST_CHECK(floating_base_state_in.acceleration.angular[i] == floating_base_state_out.acceleration.angular[i]);
    }
}

BOOST_AUTO_TEST_CASE(fk){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string world_frame = "world";
    string tip_frame = "kuka_lbr_l_tcp";

    RobotModelRBDL robot_model;
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model.configure(cfg));

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model.jointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 rbs = robot_model.rigidBodyState(world_frame, tip_frame);

    // Check correctness of space Jacobian against forward kinematics in RBDL
    base::MatrixXd space_jac = robot_model.spaceJacobian(world_frame, tip_frame);
    base::VectorXd twist = space_jac*robot_model.qd;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist[i] - rbs.twist.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(twist[i+3] - rbs.twist.angular[i]) < 1e-6);
    }
    base::Acceleration acc_bias = robot_model.spatialAccelerationBias(world_frame, tip_frame);
    base::Vector6d acc_bias_vect;
    acc_bias_vect.segment(0,3) = acc_bias.linear;
    acc_bias_vect.segment(3,3) = acc_bias.angular;
    base::VectorXd acc = space_jac * robot_model.qdd + acc_bias_vect;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc[i] - rbs.acceleration.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(acc[i+3] - rbs.acceleration.angular[i]) < 1e-6);
    }

    // Check correctness of body Jacobian against forward kinematics in RBDL
    base::MatrixXd body_jac = robot_model.bodyJacobian(world_frame, tip_frame);
    rbs.twist.linear = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.twist.linear;
    rbs.twist.angular = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.twist.angular;
    rbs.acceleration.linear = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.acceleration.linear;
    rbs.acceleration.angular = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.acceleration.angular;
    twist = body_jac*robot_model.qd;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist[i] - rbs.twist.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(twist[i+3] - rbs.twist.angular[i]) < 1e-6);
    }
    acc_bias = robot_model.spatialAccelerationBias(world_frame, tip_frame);
    acc_bias.linear = rbs.pose.orientation.inverse().toRotationMatrix() * acc_bias.linear;
    acc_bias.angular = rbs.pose.orientation.inverse().toRotationMatrix() * acc_bias.angular;
    acc_bias_vect;
    acc_bias_vect.segment(0,3) = acc_bias.linear;
    acc_bias_vect.segment(3,3) = acc_bias.angular;
    acc = body_jac * robot_model.qdd + acc_bias_vect;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc[i] - rbs.acceleration.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(acc[i+3] - rbs.acceleration.angular[i]) < 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(com){
    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";

    RobotModelRBDL robot_model;
    RobotModelConfig cfg(urdf_file);
    //cfg.floating_base = true; // TODO: Why doesn't this work with floating base?
    BOOST_CHECK(robot_model.configure(cfg));

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model.jointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::MatrixXd com_jac = robot_model.comJacobian();
    base::samples::RigidBodyStateSE3 com = robot_model.centerOfMass();

    base::Vector3d com_vel = com_jac*robot_model.qd;

    double dt = 0.002;
    for(int i = 0; i < robot_model.noOfJoints(); i++)
        joint_state_in[i].position += dt*joint_state_in[i].speed;
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 com_next = robot_model.centerOfMass();
    base::MatrixXd com_jac_next = robot_model.comJacobian();

    base::Vector3d com_diff = (1.0 / dt) * (com_next.pose.position - com.pose.position);
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_vel[i] - com.twist.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(com_vel[i] - com_diff[i]) < 1e-3);
    }
}

BOOST_AUTO_TEST_CASE(dynamics){

    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";

    RobotModelRBDL robot_model;
    RobotModelConfig cfg(urdf_file);
    cfg.floating_base = true;
    BOOST_CHECK(robot_model.configure(cfg));
    base::samples::Joints joint_state_in = makeRandomJointState(robot_model.jointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model.update(joint_state_in, floating_base_state_in));

    base::VectorXd bias_forces = robot_model.biasForces();
    base::MatrixXd mass_inertia_mat = robot_model.jointSpaceInertiaMatrix();

    cout<<"Bias forces"<<endl;
    cout<<bias_forces.transpose()<<endl;
    cout<<"Mass Inertia Matrix"<<endl;
    cout<<mass_inertia_mat<<endl;

    base::commands::Joints state;
    state.names = robot_model.jointNames();
    state.elements.resize(robot_model.noOfJoints());
    robot_model.computeInverseDynamics(state);

    cout<<"Inverse dynamics: "<<endl;
    for(auto n : state.names) cout<<state[n].effort<<" "; cout<<endl;
    base::VectorXd tau = (mass_inertia_mat * robot_model.qdd + bias_forces).segment(6,robot_model.noOfJoints());
    cout<<"Check"<<endl;
    cout<<(mass_inertia_mat * robot_model.qdd + bias_forces).segment(6,robot_model.noOfJoints()).transpose()<<endl;

    for(int i = 0; i < robot_model.noOfJoints(); i++)
        BOOST_CHECK(fabs(tau[i] - state[i].effort) < 1e-6);
}
