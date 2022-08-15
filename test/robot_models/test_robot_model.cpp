#include "test_robot_model.hpp"
#include <boost/test/unit_test.hpp>

using namespace std;

namespace wbc {
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
    floating_base_state.time = base::Time::now();
    floating_base_state.frame_id = "world";
    return floating_base_state;
}

base::Twist operator-(const base::Pose& a, const base::Pose& b){
    Eigen::Affine3d tf_a = a.toTransform();
    Eigen::Affine3d tf_b = b.toTransform();

    Eigen::Matrix3d rot_mat = tf_b.rotation().inverse() * tf_a.rotation();
    Eigen::AngleAxisd angle_axis;
    angle_axis.fromRotationMatrix(rot_mat);

    base::Twist twist;
    twist.linear = tf_a.translation() - tf_b.translation();
    twist.angular = tf_b.rotation() * (angle_axis.axis() * angle_axis.angle());
    return twist;
}

void testFK(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    double dt = 0.001;
    base::samples::Joints joint_state_in = makeRandomJointState(robot_model->actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::VectorXd q(robot_model->noOfJoints());
    base::VectorXd qd(robot_model->noOfJoints());
    base::VectorXd qdd(robot_model->noOfJoints());
    robot_model->systemState(q,qd,qdd);

    base::samples::RigidBodyStateSE3 rbs = robot_model->rigidBodyState(robot_model->worldFrame(), tip_frame);

    // Integrate one step
    for(int i = 0; i < joint_state_in.size(); i++){
        joint_state_in[i].position += dt * joint_state_in[i].speed;
        joint_state_in[i].speed += dt * joint_state_in[i].acceleration;
    }
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 rbs_next = robot_model->rigidBodyState(robot_model->worldFrame(), tip_frame);

    // Check correctness of FK solution against integrated pose/velocity
    base::Twist diff_vel = rbs_next.pose - rbs.pose;
    diff_vel.linear = diff_vel.linear / dt;
    diff_vel.angular = diff_vel.angular / dt;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(diff_vel.linear[i] - rbs.twist.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(diff_vel.angular[i] - rbs.twist.angular[i]) < 1e-6);
    }

    if(verbose){
        cout<<"FK"<<endl;
        printRbs(rbs);
        cout<<"Diff Vel"<<endl;
        cout<<diff_vel.linear<<" "<<diff_vel.angular<<endl;
    }
}

void testSpaceJacobian(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model->actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::VectorXd q(robot_model->noOfJoints());
    base::VectorXd qd(robot_model->noOfJoints());
    base::VectorXd qdd(robot_model->noOfJoints());
    robot_model->systemState(q,qd,qdd);

    cout<<"Joint Names"<<endl;
    for(auto n : robot_model->jointNames())
        cout<<n<<" ";

    cout<<"Actuated joint Names"<<endl;
    for(auto n : robot_model->actuatedJointNames())
        cout<<n<<" ";
    cout<<endl;

    cout<<"Independent joint Names"<<endl;
    for(auto n : robot_model->independentJointNames())
        cout<<n<<" ";
    cout<<endl;

    cout<<"Joint state"<<endl;
    for(auto n : joint_state_in.names)
        cout<<n<<" "<<joint_state_in[n].position<<" "<<joint_state_in[n].speed<<" "<<joint_state_in[n].acceleration<<" "<<endl;

    cout<<q.transpose()<<endl;
    cout<<qd.transpose()<<endl;
    cout<<qdd.transpose()<<endl;

    base::samples::RigidBodyStateSE3 rbs = robot_model->rigidBodyState(robot_model->worldFrame(), tip_frame);

    // Check correctness of FK solution agains Js*qd
    base::MatrixXd Js = robot_model->spaceJacobian(robot_model->worldFrame(), tip_frame);
    base::VectorXd twist = Js*qd;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist[i] - rbs.twist.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(twist[i+3] - rbs.twist.angular[i]) < 1e-3);
    }
    // Check correctness of FK solution agains Js*qdd + Js_dot*qd
    base::Acceleration acc_bias = robot_model->spatialAccelerationBias(robot_model->worldFrame(), tip_frame);
    base::Vector6d acc_bias_vect;
    acc_bias_vect.segment(0,3) = acc_bias.linear;
    acc_bias_vect.segment(3,3) = acc_bias.angular;
    base::VectorXd acc = Js * qdd + acc_bias_vect;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc[i] - rbs.acceleration.linear[i]) < 1e-3);
        BOOST_CHECK(fabs(acc[i+3] - rbs.acceleration.angular[i]) < 1e-3);
    }

    if(verbose){
        cout<<"FK"<<endl;
        printRbs(rbs);
        cout<<"Twist"<<endl;
        cout<<twist.transpose()<<endl;
        cout<<"Acc"<<endl;
        cout<<acc.transpose()<<endl;
        cout<<"Space Jac"<<endl;
        cout<<Js<<endl;
    }
}

void testBodyJacobian(RobotModelPtr robot_model, const string &tip_frame, bool verbose){

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model->actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::VectorXd q(robot_model->noOfJoints());
    base::VectorXd qd(robot_model->noOfJoints());
    base::VectorXd qdd(robot_model->noOfJoints());
    robot_model->systemState(q,qd,qdd);

    base::samples::RigidBodyStateSE3 rbs = robot_model->rigidBodyState(robot_model->worldFrame(), tip_frame);

    // Check correctness of body Jacobian against forward kinematics in RBDL
    base::MatrixXd Jb = robot_model->bodyJacobian(robot_model->worldFrame(), tip_frame);
    rbs.twist.linear = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.twist.linear;
    rbs.twist.angular = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.twist.angular;
    rbs.acceleration.linear = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.acceleration.linear;
    rbs.acceleration.angular = rbs.pose.orientation.inverse().toRotationMatrix() * rbs.acceleration.angular;
    base::VectorXd twist = Jb*qd;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(twist[i] - rbs.twist.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(twist[i+3] - rbs.twist.angular[i]) < 1e-6);
    }
    base::Acceleration acc_bias = robot_model->spatialAccelerationBias(robot_model->worldFrame(), tip_frame);
    acc_bias.linear = rbs.pose.orientation.inverse().toRotationMatrix() * acc_bias.linear;
    acc_bias.angular = rbs.pose.orientation.inverse().toRotationMatrix() * acc_bias.angular;
    base::Vector6d acc_bias_vect;
    acc_bias_vect.segment(0,3) = acc_bias.linear;
    acc_bias_vect.segment(3,3) = acc_bias.angular;
    base::VectorXd acc = Jb * qdd + acc_bias_vect;
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc[i] - rbs.acceleration.linear[i]) < 1e-6);
        BOOST_CHECK(fabs(acc[i+3] - rbs.acceleration.angular[i]) < 1e-6);
    }
    if(verbose){
        cout<<"FK"<<endl;
        printRbs(rbs);
        cout<<"Twist"<<endl;
        cout<<twist.transpose()<<endl;
        cout<<"Acc"<<endl;
        cout<<acc.transpose()<<endl;
        cout<<"Body Jac"<<endl;
        cout<<Jb<<endl;
    }
}

void testCoMJacobian(RobotModelPtr robot_model, bool verbose){

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model->actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::MatrixXd com_jac = robot_model->comJacobian();
    base::samples::RigidBodyStateSE3 com = robot_model->centerOfMass();

    base::VectorXd q,qd,qdd;
    robot_model->systemState(q,qd,qdd);

    base::Vector3d com_vel = com_jac*qd;

    double dt = 0.002;
    for(int i = 0; i < robot_model->noOfActuatedJoints(); i++)
        joint_state_in[i].position += dt*joint_state_in[i].speed;
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::samples::RigidBodyStateSE3 com_next = robot_model->centerOfMass();

    base::Vector3d com_vel_diff = (1.0 / dt) * (com_next.pose.position - com.pose.position);
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
void testDynamics(RobotModelPtr robot_model, bool verbose){

    base::samples::Joints joint_state_in = makeRandomJointState(robot_model->actuatedJointNames());
    base::samples::RigidBodyStateSE3 floating_base_state_in = makeRandomFloatingBaseState();
    BOOST_CHECK_NO_THROW(robot_model->update(joint_state_in, floating_base_state_in));

    base::VectorXd C = robot_model->biasForces();
    base::MatrixXd Hq = robot_model->jointSpaceInertiaMatrix();

    base::commands::Joints state;
    state.names = robot_model->actuatedJointNames();
    state.elements.resize(robot_model->noOfActuatedJoints());
    robot_model->computeInverseDynamics(state);

    base::VectorXd q,qd,qdd;
    robot_model->systemState(q,qd,qdd);
    uint start_idx = 0;
    if(robot_model->hasFloatingBase())
        start_idx = 6;

    base::VectorXd tau = (Hq*qdd + C).segment(start_idx,robot_model->noOfActuatedJoints());
    for(int i = 0; i < robot_model->noOfActuatedJoints(); i++)
        BOOST_CHECK(fabs(tau[i] - state[i].effort) < 1e-3);

    if(verbose){
        cout<<"Bias forces"<<endl;
        cout<<C.transpose()<<endl;
        cout<<"Mass Inertia Matrix"<<endl;
        cout<<Hq<<endl;
        cout<<"Inverse dynamics solution tau: "<<endl;
        for(auto n : state.names) cout<<state[n].effort<<" "; cout<<endl;
        cout<<"Check Hq*qdd + C == tau"<<endl;
        cout<<tau.transpose()<<endl;
    }
}

}
