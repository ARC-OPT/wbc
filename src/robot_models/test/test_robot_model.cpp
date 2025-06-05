#include "test_robot_model.hpp"
#include <boost/test/unit_test.hpp>

using namespace std;

namespace wbc {
void printRbs(types::RigidBodyState rbs){
    cout<<"Position:      "<<rbs.pose.position.transpose()<<endl;
    cout<<"Orientation:   "<<rbs.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist linear:  "<<rbs.twist.linear.transpose()<<endl;
    cout<<"Twist angular: "<<rbs.twist.angular.transpose()<<endl;
    cout<<"Acc linear:    "<<rbs.acceleration.linear.transpose()<<endl;
    cout<<"Acc angular:   "<<rbs.acceleration.angular.transpose()<<endl<<endl;
}

types::JointState makeRandomJointState(uint n){
    types::JointState joint_state;
    joint_state.resize(n);
    for(uint i = 0; i < n; i++){
        joint_state.position[i] = (double)rand()/RAND_MAX;
        joint_state.velocity[i] = (double)rand()/RAND_MAX;
        joint_state.acceleration[i] = (double)rand()/RAND_MAX;
    }
    return joint_state;
}

types::RigidBodyState makeRandomFloatingBaseState(){
    types::RigidBodyState floating_base_state;
    floating_base_state.pose.position = Eigen::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear  = Eigen::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.linear  = Eigen::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.twist.angular = Eigen::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.angular = Eigen::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    return floating_base_state;
}

void testFK(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    double dt = 0.001;

    types::JointState joint_state = makeRandomJointState(robot_model->na());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::Pose pose = robot_model->pose(tip_frame);
    types::Twist twist = robot_model->twist(tip_frame);

    // Integrate one step. TODO: This does not work for floating base robots
    joint_state.position += dt * joint_state.velocity;
    joint_state.velocity += dt * joint_state.acceleration;
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::Pose pose_next = robot_model->pose(tip_frame);

    // Check correctness of FK solution against integrated pose/velocity
    types::Twist diff_vel = pose_next - pose;
    diff_vel.linear = diff_vel.linear / dt;
    diff_vel.angular = diff_vel.angular / dt;

    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(diff_vel.linear[i] - twist.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(diff_vel.angular[i] - twist.angular[i]) < 1e-3);
    }

    if(verbose){
        cout<<"FK"<<endl;
        cout<<"Position:      "<<pose.position.transpose()<<endl;
        cout<<"Orientation:   "<<pose.orientation.coeffs().transpose()<<endl;
        cout<<"Twist linear:  "<<twist.linear.transpose()<<endl;
        cout<<"Twist angular: "<<twist.angular.transpose()<<endl<<endl;
        cout<<"Diff Vel"<<endl;
        cout<<diff_vel.linear<<" "<<diff_vel.angular<<endl;
    }
}

void testSpaceJacobian(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    types::JointState joint_state = makeRandomJointState(robot_model->na());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::Twist twist = robot_model->twist(tip_frame);
    types::SpatialAcceleration acceleration = robot_model->acceleration(tip_frame);

    // Check correctness of FK solution agains Js*qd
    Eigen::MatrixXd Js = robot_model->spaceJacobian(tip_frame);
    Eigen::VectorXd twist_eigen = Js*robot_model->getQd();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist_eigen[i] - twist.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(twist_eigen[i+3] - twist.angular[i]) < 1e-3);
    }
    // Check correctness of FK solution agains Js*qdd + Js_dot*qd
    types::SpatialAcceleration acc_bias = robot_model->spatialAccelerationBias(tip_frame);
    Eigen::VectorXd acc = Js * robot_model->getQdd() + acc_bias.vector6d();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc[i] - acceleration.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(acc[i+3] - acceleration.angular[i]) < 1e-3);
    }

    if(verbose){
        cout<<"FK"<<endl;
        cout<<"Twist linear:  "<<twist.linear.transpose()<<endl;
        cout<<"Twist angular: "<<twist.angular.transpose()<<endl;
        cout<<"Acc linear:    "<<acceleration.linear.transpose()<<endl;
        cout<<"Acc angular:   "<<acceleration.angular.transpose()<<endl<<endl;
        cout<<"Twist"<<endl;
        cout<<twist_eigen.transpose()<<endl;
        cout<<"Acc"<<endl;
        cout<<acc.transpose()<<endl;
        cout<<"Space Jac"<<endl;
        cout<<Js<<endl;
    }
}

void testBodyJacobian(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    types::JointState joint_state = makeRandomJointState(robot_model->na());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::Pose pose = robot_model->pose(tip_frame);
    types::Twist twist = robot_model->twist(tip_frame);
    types::SpatialAcceleration acceleration = robot_model->acceleration(tip_frame);

    // Check correctness of body Jacobian against forward kinematics in RBDL
    Eigen::MatrixXd Jb = robot_model->bodyJacobian(tip_frame);
    twist.linear = pose.orientation.inverse().toRotationMatrix() * twist.linear;
    twist.angular = pose.orientation.inverse().toRotationMatrix() * twist.angular;
    acceleration.linear = pose.orientation.inverse().toRotationMatrix() * acceleration.linear;
    acceleration.angular = pose.orientation.inverse().toRotationMatrix() * acceleration.angular;
    Eigen::VectorXd twist_eigen = Jb*robot_model->getQd();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist_eigen[i] - twist.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(twist_eigen[i+3] - twist.angular[i]) < 1e-6);
    }
    types::SpatialAcceleration acc_bias = robot_model->spatialAccelerationBias(tip_frame);
    acc_bias.linear = pose.orientation.inverse().toRotationMatrix() * acc_bias.linear;
    acc_bias.angular = pose.orientation.inverse().toRotationMatrix() * acc_bias.angular;
    Eigen::VectorXd acc_eigen = Jb * robot_model->getQdd() + acc_bias.vector6d();
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_eigen[i] - acceleration.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(acc_eigen[i+3] - acceleration.angular[i]) < 1e-6);
    }
    if(verbose){
        cout<<"FK"<<endl;
        cout<<"Twist linear:  "<<twist.linear.transpose()<<endl;
        cout<<"Twist angular: "<<twist.angular.transpose()<<endl;
        cout<<"Acc linear:    "<<acceleration.linear.transpose()<<endl;
        cout<<"Acc angular:   "<<acceleration.angular.transpose()<<endl<<endl;
        cout<<"Twist"<<endl;
        cout<<twist_eigen.transpose()<<endl;
        cout<<"Acc"<<endl;
        cout<<acc_eigen.transpose()<<endl;
        cout<<"Body Jac"<<endl;
        cout<<Jb<<endl;
    }
}

void testCoMJacobian(RobotModelPtr robot_model, bool verbose){

    types::JointState joint_state = makeRandomJointState(robot_model->na());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    Eigen::MatrixXd com_jac = robot_model->comJacobian();
    types::RigidBodyState com = robot_model->centerOfMass();

    Eigen::Vector3d com_vel = com_jac*robot_model->getQd();

    double dt = 0.002;
    joint_state.position += dt*joint_state.velocity;
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    types::RigidBodyState com_next = robot_model->centerOfMass();

    Eigen::Vector3d com_vel_diff = (1.0 / dt) * (com_next.pose.position - com.pose.position);
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_vel[i] - com_vel_diff[i]) < 1e-3);
        BOOST_CHECK(fabs(com_vel[i] - com_vel_diff[i]) < 1e-3);
    }

    if(verbose){
        cout<<"CoM"<<endl;
        printRbs(com);
        cout<<"Vel diff"<<endl;
        cout<<com_vel_diff.transpose()<<endl;
        cout<<"CoM Jacobian"<<endl;
        cout<<com_jac<<endl;
    }

}

void testInverseDynamics(RobotModelPtr robot_model, bool verbose){

    types::JointState joint_state = makeRandomJointState(robot_model->na());
    types::RigidBodyState floating_base_state = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state.position,
                                             joint_state.velocity,
                                             joint_state.acceleration,
                                             floating_base_state.pose,
                                             floating_base_state.twist,
                                             floating_base_state.acceleration));

    Eigen::VectorXd tau = robot_model->inverseDynamics();
    Eigen::VectorXd tau_test = (robot_model->jointSpaceInertiaMatrix()*robot_model->getQdd() + robot_model->biasForces());

    BOOST_CHECK(tau.size() == tau_test.size());

    for(int i = 0; i < tau.size(); i++)
        BOOST_CHECK(fabs(tau[i] - tau_test[i]) < 1e-3);

    if(verbose){
        cout<<"Tau: "<<endl;
        cout<<tau.transpose()<<endl;
        cout<<"Tau test: "<<endl;
        cout<<tau_test.transpose()<<endl;
    }
}

}
