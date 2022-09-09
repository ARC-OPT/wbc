#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <hyrodyn/robot_model_hyrodyn.hpp>
#include "tools/URDFTools.hpp"
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <base/samples/Joints.hpp>

using namespace std;
using namespace hyrodyn;

void printRbs(base::samples::RigidBodyStateSE3 rbs){
    cout<<"Position:      "<<rbs.pose.position.transpose()<<endl;
    cout<<"Orientation:   "<<rbs.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist linear:  "<<rbs.twist.linear.transpose()<<endl;
    cout<<"Twist angular: "<<rbs.twist.angular.transpose()<<endl;
    cout<<"Acc linear:    "<<rbs.acceleration.linear.transpose()<<endl;
    cout<<"Acc angular:   "<<rbs.acceleration.angular.transpose()<<endl<<endl;
}

base::Matrix3d getRotMatfromRPY(const base::Vector3d& rpy)
{
    // Returns the Rotation matrix (3x3) by applying Euler angles in the order: roll[0], pitch[1], yaw[2]
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    base::Matrix3d R_z, R_y, R_x;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
                  0,         0, 1;
    R_y << cos(pitch), 0, sin(pitch),
                    0, 1,          0,
          -sin(pitch), 0, cos(pitch);
    R_x << 1,         0,          0,
           0, cos(roll), -sin(roll),
           0, sin(roll),  cos(roll);

    return R_z*R_y*R_x;
}

base::Vector3d getEulerAngles(const base::Quaterniond& orientation)
{
    // Returns the Euler angles in the order: roll[0], pitch[1], yaw[2]
    // Inspired from Determining yaw, pitch, and roll from a rotation matrix (http://planning.cs.uiuc.edu/node103.html) and Pose.cpp from base-types
    const Eigen::Matrix3d m = orientation.toRotationMatrix();
    //cout<<"Input RotMat from Quat: \n"<<m<<endl;
    double x = Eigen::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();	// x = sqrt(r33^2 + r32^2)
    base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);	// pitch = atan2(-r31, x)
    if (x > Eigen::NumTraits<double>::dummy_precision()){
        res[2] = ::atan2(m.coeff(1,0), m.coeff(0,0));	// yaw = atan2(r21, r11)
        res[0] = ::atan2(m.coeff(2,1), m.coeff(2,2));	// roll = atan2(r32, r33)
    }else{
        res[2] = 0;
        res[0] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
    }

    //cout<<"Output RotMat from RPY: \n"<<getRotMat(res)<<endl;
    res = -res;	// this produces correct derivatives of floating base coordinates and lower error in inverse dynamics (not clear, why?)
    return res;
}

base::Vector3d getEulerAnglesDerivative(const base::Quaterniond& orientation, const base::Vector3d& omega)
{
    // Returns the time derivative of Euler angles in the order: yaw[0], pitch[1], roll[2]

    Eigen::Vector3d rpy = getEulerAngles(orientation);
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Eigen::Matrix3d m;
     // Method 1: this formulation is motivated from body fixed angular velocity, see https://davidbrown3.github.io/2017-07-25/EulerAngles/
    m << cos(yaw)/cos(pitch), -sin(yaw)/cos(pitch), 0,
         sin(yaw),             cos(yaw),            0,
        -cos(yaw)*sin(pitch)/cos(pitch), sin(yaw)*sin(pitch)/cos(pitch), 1;
    //m = -m; // correction term  (it produces correct derivatives of the floating base coordinates as well but error in inverse dynamics is high)

    /*	// Method 2: Inspired from Differential rotations (http://planning.cs.uiuc.edu/node690.html), corrected by Khalil's book
    m << cos(yaw)/cos(pitch), sin(yaw)/cos(pitch), 0,
         -sin(yaw),           cos(yaw),            0,
         cos(yaw)*sin(pitch)/cos(pitch), sin(yaw)*sin(pitch)/cos(pitch), 1;
    */
    // rpy_dot = A * omega, where A is the Jacobian that maps the angular velocity(omega) to RPY angular rates
    base::Vector3d rpy_dot = m*omega;

    return rpy_dot;
}


base::Vector3d getEulerAnglesSecondDerivative(const base::Quaterniond& orientation, const base::Vector3d& omega, const base::Vector3d& omega_dot)
{
    // Returns the 2nd time derivative of Euler angles in the order: yaw[0], pitch[1], roll[2]
    // 1. Extract RPY angles from Quaternion representation
    Eigen::Vector3d rpy = getEulerAngles(orientation);
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);
    // 2. Extract RPY angle rates from Quaternion and angular velocity
    base::Vector3d rpy_dot = getEulerAnglesDerivative(orientation, omega);
    double roll_dot = rpy_dot(0);
    double pitch_dot = rpy_dot(1);
    double yaw_dot = rpy_dot(2);

    // 3. Extract RPY 2nd derivative (symbolic derivative derived in MATLAB symbolic toolbox of Method 1)
    double t2 = cos(pitch);
    double t3 = cos(yaw);
    double t4 = sin(pitch);
    double t5 = sin(yaw);
    double t6 = t4*t4;
    double t7 = 1.0/t2;
    double t8 = t7*t7;
    base::Matrix3d m;
    m(0,0) = -t5*t7*yaw_dot+pitch_dot*t3*t4*t8;
    m(0,1) = -t3*t7*yaw_dot-pitch_dot*t4*t5*t8;
    m(0,2) = 0.0;
    m(1,0) = t3*yaw_dot;
    m(1,1) = -t5*yaw_dot;
    m(1,2) = 0.0;
    m(2,0) = -pitch_dot*t3-pitch_dot*t3*t6*t8+t4*t5*t7*yaw_dot;
    m(2,1) = pitch_dot*t5+pitch_dot*t5*t6*t8+t3*t4*t7*yaw_dot;
    m(2,2) = 0.0;
    //m = -m;	// correction term (it produces correct derivatives of the floating base coordinates as well but error in inverse dynamics is high)

    /*
    // additional sign changes as per lecture notes (Method 2)
    m(0,1) = -m(0,1);
    m(1,0) = -m(1,0);
    m(2,0) = -m(2,0);
    */
    // rpy_ddot = A * omega_dot + Adot * omega
    base::Vector3d rpy_ddot = getEulerAnglesDerivative(orientation, omega_dot) + m*omega;

    return rpy_ddot;
}

base::samples::RigidBodyStateSE3 testRobotModelKDL(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state, const base::samples::RigidBodyStateSE3& floating_base_state){
    string root_frame = wbc::URDFTools::rootLinkFromURDF(urdf_file);

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(urdf_file,tree)){
        cerr<<"Failed to load tree from urdf file "<<urdf_file<<endl;
        abort();
    }

    uint nj = joint_state.size();

    // Setup solvers
    KDL::Chain chain;
    tree.getChain(root_frame,tip_frame,chain);
    KDL::ChainFkSolverVel_recursive solver(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);
    KDL::ChainJntToJacDotSolver jac_dot_solver(chain);
    jac_dot_solver.setRepresentation(KDL::ChainJntToJacDotSolver::HYBRID);

    // Setup KDL state vectors
    KDL::JntArrayVel q_and_qd_kdl(nj+6);
    KDL::JntArrayAcc qdd_kdl(nj+6);
    base::Vector3d euler = getEulerAngles(floating_base_state.pose.orientation);//.toRotationMatrix().eulerAngles(0, 1, 2);

    base::Vector3d omega(floating_base_state.twist.angular[0], floating_base_state.twist.angular[1], floating_base_state.twist.angular[2]);
    base::Vector3d rpy_dot = getEulerAnglesDerivative(floating_base_state.pose.orientation, omega);

    base::Vector3d omega_dot(floating_base_state.acceleration.angular[0], floating_base_state.acceleration.angular[1], floating_base_state.acceleration.angular[2]);
    base::Vector3d rpy_ddot = getEulerAnglesSecondDerivative(floating_base_state.pose.orientation, omega, omega_dot);

    for(int i = 0; i < 3; i++){
        q_and_qd_kdl.q(i) = floating_base_state.pose.position[i];
        q_and_qd_kdl.qdot(i) = qdd_kdl.qdot(i) = floating_base_state.twist.linear[i];
        qdd_kdl.qdotdot(i) = floating_base_state.acceleration.linear[i];
        q_and_qd_kdl.q(i+3) = euler[i];
        q_and_qd_kdl.qdot(i+3) = qdd_kdl.qdot(i+3) = rpy_dot[i];
        qdd_kdl.qdotdot(i+3) = rpy_ddot[i];
    }

    base::Matrix3d rot_mat = base::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();;
    q_and_qd_kdl.qdot(4) = qdd_kdl.qdot(4) = (rot_mat.inverse()*floating_base_state.twist.angular)[4];

    for(int i = 0; i < nj; i++){
        q_and_qd_kdl.q(i+6) = joint_state[i].position;
        q_and_qd_kdl.qdot(i+6) = qdd_kdl.qdot(i) = joint_state[i].speed;
        qdd_kdl.qdotdot(i+6) = joint_state[i].acceleration;
    }

    std::cout<<"Joint configuration KDL"<<std::endl;
    cout<<q_and_qd_kdl.q.data.transpose()<<endl;
    cout<<q_and_qd_kdl.qdot.data.transpose()<<endl;
    cout<<qdd_kdl.qdotdot.data.transpose()<<endl<<endl;

    // Compute pose & twist
    KDL::FrameVel frame_vel;
    if(solver.JntToCart(q_and_qd_kdl,frame_vel)){
        cerr<<"Failed to compute forward kinematics for chain "<<root_frame<<" -> "<<tip_frame<<endl;
        abort();
    }
    KDL::Frame pose_kdl = frame_vel.GetFrame();
    KDL::Twist twist_kdl = frame_vel.GetTwist();

    // Compute acc
    KDL::Jacobian jac_kdl(nj+6);
    if(jac_solver.JntToJac(q_and_qd_kdl.q, jac_kdl)){
        cerr<<"Failed to compute Jacobian for chain "<<root_frame<<" -> "<<tip_frame<<endl;
        abort();
    }
    KDL::Jacobian jac_dot_kdl(nj+6);
    if(jac_dot_solver.JntToJacDot(q_and_qd_kdl, jac_dot_kdl)){
        cerr<<"Failed to compute Jacobian Dot for chain "<<root_frame<<" -> "<<tip_frame<<endl;
        abort();
    }
    base::VectorXd acc_kdl = jac_dot_kdl.data*q_and_qd_kdl.qdot.data + jac_kdl.data*qdd_kdl.qdotdot.data;

    base::samples::RigidBodyStateSE3 rbs;
    rbs.pose.position << pose_kdl.p(0), pose_kdl.p(1), pose_kdl.p(2);
    double x, y, z, w;
    pose_kdl.M.GetQuaternion(x, y, z, w);
    rbs.pose.orientation = base::Quaterniond(w, x, y, z);
    rbs.twist.linear  << twist_kdl.vel(0), twist_kdl.vel(1), twist_kdl.vel(2);
    rbs.twist.angular << twist_kdl.rot(0), twist_kdl.rot(1), twist_kdl.rot(2);
    rbs.acceleration.linear = acc_kdl.segment(0,3);
    rbs.acceleration.angular = acc_kdl.segment(3,3);
    return rbs;
}

base::samples::RigidBodyStateSE3 testRobotModelRBDL(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state, const base::samples::RigidBodyStateSE3& floating_base_state){

    RigidBodyDynamics::Model rbdl_model;
    if(!Addons::URDFReadFromFile(urdf_file.c_str(), &rbdl_model, true))
        abort();

    // Transformation from fb body linear acceleration to fb joint linear acceleration ----------------------
    // since RBDL treat the floating base as XYZ translation followed by spherica
    // aj is S*qdd(i)
    // j is parent of i
    // a(i) = Xa(j) + aj + cross(v(i), vj)
    // For pinocchio we give directly a(fb), for RBDL we give aj instead (for the first two joints)
    // so we have to remove the cross contribution cross(v(i), vj) from it

    Eigen::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();

    base::Twist fb_twist = floating_base_state.twist;
    base::Acceleration fb_acc = floating_base_state.acceleration;

    Eigen::VectorXd spherical_j_vel(6);
    spherical_j_vel << fb_twist.angular, Eigen::Vector3d::Zero();
    Eigen::VectorXd spherical_b_vel(6);
    spherical_b_vel << fb_twist.angular, fb_rot.transpose() * fb_twist.linear;

    Eigen::VectorXd fb_spherical_cross = crossm(spherical_b_vel, spherical_j_vel);

    fb_acc.linear = fb_acc.linear - fb_rot * fb_spherical_cross.tail<3>(); // remove cross contribution from linear acc s(in world coordinates as RBDL want)
    // ------------------------------------------------------------------------------------------------------------

    // Setup RBDL state vectors
    uint nj = joint_state.size();
    // RBDL adds the real part of the quaternion for the floating base at the end of the state vector, so we have to augment the state vector by one here!
    Eigen::VectorXd q(rbdl_model.dof_count+1), qd(rbdl_model.dof_count), qdd(rbdl_model.dof_count);
    for(int i = 0; i < nj; i++){
        q[i+6] = joint_state[i].position;
        qd[i+6] = joint_state[i].speed;
        qdd[i+6] = joint_state[i].acceleration;
    }
    for(int i = 0; i < 3; i++){
        q[i] = floating_base_state.pose.position[i];
        qd[i] = fb_twist.linear[i];
        qdd[i] = fb_acc.linear[i];
        qd[i+3] = fb_twist.angular[i];
        qdd[i+3] = fb_acc.angular[i];
    }
    int floating_body_id = rbdl_model.GetBodyId(wbc::URDFTools::rootLinkFromURDF(urdf_file).c_str());
    rbdl_model.SetQuaternion(floating_body_id, Math::Quaternion(floating_base_state.pose.orientation.coeffs()), q);

    std::cout<<"Joint configuration RBDL"<<std::endl;
    cout<<q.transpose()<<endl;
    cout<<qd.transpose()<<endl;
    cout<<qdd.transpose()<<endl<<endl;

    base::samples::RigidBodyStateSE3 rbs;
    int body_id = rbdl_model.GetBodyId(tip_frame.c_str());
    rbs.pose.position = CalcBodyToBaseCoordinates(rbdl_model,q,body_id,base::Vector3d(0,0,0));
    rbs.pose.orientation = base::Quaterniond(CalcBodyWorldOrientation(rbdl_model,q,body_id).inverse());
    Math::SpatialVector twist_rbdl = CalcPointVelocity6D(rbdl_model, q, qd, body_id, base::Vector3d(0,0,0));
    Math::SpatialVector acc_rbdl = CalcPointAcceleration6D(rbdl_model,q,qd,qdd,body_id, base::Vector3d(0,0,0));
    rbs.twist.linear = twist_rbdl.segment(3,3);
    rbs.twist.angular = twist_rbdl.segment(0,3);
    rbs.acceleration.linear = acc_rbdl.segment(3,3);
    rbs.acceleration.angular = acc_rbdl.segment(0,3);

    base::Vector3d point_position;
    point_position.setZero();
    Math::MatrixNd J;
    J.setZero(6, nj+6);
    CalcBodySpatialJacobian(rbdl_model,q,body_id,J);

    base::MatrixXd jac(6,nj+6);
    jac.block(0,0,3,nj+6) = J.block(3,0,3,nj+6);
    jac.block(3,0,3,nj+6) = J.block(0,0,3,nj+6);


    cout<<"Jac RBDL"<<endl;
    cout<<jac<<endl;

    return rbs;
}


base::samples::RigidBodyStateSE3 testRobotModelHyrodyn(const string &urdf_file, const string &sub_mec_file, const string &tip_frame, const base::samples::Joints &joint_state, const base::samples::RigidBodyStateSE3& floating_base_state){

    RobotModel_HyRoDyn hyrodyn;
    hyrodyn.load_robotmodel(urdf_file, sub_mec_file);

    uint nj = joint_state.size();

    // Transformation from fb body linear acceleration to fb joint linear acceleration ----------------------
    // look at RBDL for description
    Eigen::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();

    base::Twist fb_twist = floating_base_state.twist;
    base::Acceleration fb_acc = floating_base_state.acceleration;

    Eigen::VectorXd spherical_j_vel(6);
    spherical_j_vel << fb_twist.angular, Eigen::Vector3d::Zero();
    Eigen::VectorXd spherical_b_vel(6);
    spherical_b_vel << fb_twist.angular, fb_rot.transpose() * fb_twist.linear;

    Eigen::VectorXd fb_spherical_cross = crossm(spherical_b_vel, spherical_j_vel);
    fb_acc.linear = fb_acc.linear - fb_rot * fb_spherical_cross.tail<3>(); // remove cross contribution from linear acc s(in world coordinates as RBDL want)
    // --------------------------------------------------------------------------------------------------

    hyrodyn.floating_robot_pose.segment(0,3) = floating_base_state.pose.position;
    hyrodyn.floating_robot_pose[3] = floating_base_state.pose.orientation.x();
    hyrodyn.floating_robot_pose[4] = floating_base_state.pose.orientation.y();
    hyrodyn.floating_robot_pose[5] = floating_base_state.pose.orientation.z();
    hyrodyn.floating_robot_pose[6] = floating_base_state.pose.orientation.w();
    hyrodyn.floating_robot_twist.segment(0,3) = floating_base_state.twist.angular;
    hyrodyn.floating_robot_twist.segment(3,3) = floating_base_state.twist.linear;
    hyrodyn.floating_robot_accn.segment(0,3) = fb_acc.angular;
    hyrodyn.floating_robot_accn.segment(3,3) = fb_acc.linear;

    for( unsigned int i = 0; i < nj; ++i){
        hyrodyn.y_robot[i]   = joint_state[i].position;
        hyrodyn.yd_robot[i]  = joint_state[i].speed;
        hyrodyn.ydd_robot[i] = joint_state[i].acceleration;
    }
    hyrodyn.update_all_independent_coordinates();


    std::cout<<"Joint configuration Hyrodyn"<<std::endl;
    cout<<hyrodyn.y.transpose()<<endl;
    cout<<hyrodyn.yd.transpose()<<endl;
    cout<<hyrodyn.ydd.transpose()<<endl<<endl;

    base::samples::RigidBodyStateSE3 rbs;
    hyrodyn.calculate_forward_kinematics(tip_frame);
    rbs.pose.position        = hyrodyn.pose.segment(0,3);
    rbs.pose.orientation     = base::Quaterniond(hyrodyn.pose[6],hyrodyn.pose[3],hyrodyn.pose[4],hyrodyn.pose[5]);
    rbs.twist.linear         = hyrodyn.twist.segment(3,3);
    rbs.twist.angular        = hyrodyn.twist.segment(0,3);
    rbs.acceleration.linear  = hyrodyn.spatial_acceleration.segment(3,3);
    rbs.acceleration.angular = hyrodyn.spatial_acceleration.segment(0,3);

    hyrodyn.calculate_space_jacobian(tip_frame);
    base::MatrixXd jac(6,nj+6);
    jac.block(0,0,3,nj+6) = hyrodyn.Js.block(3,0,3,nj+6);
    jac.block(3,0,3,nj+6) = hyrodyn.Js.block(0,0,3,nj+6);

    cout<<"Jac Hyrodyn"<<endl;
    cout<<jac<<endl;

    return rbs;
}


base::samples::RigidBodyStateSE3 testRobotModelPinocchio(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state, const base::samples::RigidBodyStateSE3& floating_base_state){
    urdf::ModelInterfaceSharedPtr robot_urdf = urdf::parseURDFFile(urdf_file);
    if(!robot_urdf)
        abort();
    pinocchio::Model model;
    pinocchio::urdf::buildModel(robot_urdf,pinocchio::JointModelFreeFlyer(), model);

    // Setup Pinocchio state vectors
    uint nj = joint_state.size();
    Eigen::VectorXd q(nj+7),qd(nj+6),qdd(nj+6);

    base::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();

    base::Twist fb_twist = floating_base_state.twist;
    fb_twist.linear = fb_rot.transpose() * floating_base_state.twist.linear;

    base::Acceleration fb_acc = floating_base_state.acceleration;
    fb_acc.linear = fb_rot.transpose() * floating_base_state.acceleration.linear;

    for(int i = 0; i < 3; i++){
        q[i] = floating_base_state.pose.position[i];
        qd[i] = fb_twist.linear[i];
        qd[i+3] = fb_twist.angular[i];
        qdd[i] = fb_acc.linear[i];
        qdd[i+3] = fb_acc.angular[i];
    }
    q[3] = floating_base_state.pose.orientation.x();
    q[4] = floating_base_state.pose.orientation.y();
    q[5] = floating_base_state.pose.orientation.z();
    q[6] = floating_base_state.pose.orientation.w();

    for(int i = 0; i < nj; i++){
        q[i+7] = joint_state[i].position;
        qd[i+6] = joint_state[i].speed;
        qdd[i+6] = joint_state[i].acceleration;
    }

    std::cout<<"Joint configuration Pinocchio"<<std::endl;
    cout<<q.transpose()<<endl;
    cout<<qd.transpose()<<endl;
    cout<<qdd.transpose()<<endl<<endl;

    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model,data,q,qd,qdd);
    pinocchio::updateFramePlacement(model,data,model.getFrameId(tip_frame));
    base::samples::RigidBodyStateSE3 rbs;
    rbs.pose.position = data.oMf[model.getFrameId(tip_frame)].translation();
    rbs.pose.orientation = base::Quaterniond(data.oMf[model.getFrameId(tip_frame)].rotation());
    rbs.twist.linear = pinocchio::getFrameVelocity(model, data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.twist.angular = pinocchio::getFrameVelocity(model, data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();
    rbs.acceleration.linear = pinocchio::getFrameClassicalAcceleration(model, data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    rbs.acceleration.angular = pinocchio::getFrameClassicalAcceleration(model, data, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED).angular();

    base::MatrixXd jac(6,q.size()-1);
    jac.setZero();
    pinocchio::computeFrameJacobian(model, data, q, model.getFrameId(tip_frame), pinocchio::LOCAL_WORLD_ALIGNED, jac);

    cout<<"Jac Pinocchio"<<endl;
    cout<<jac<<endl;

    return rbs;
}

int main(){

    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string urdf_file_with_floating_base = "../../../../models/kuka/urdf/kuka_iiwa_with_floating_base.urdf";
    string sub_mec_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    string tip_frame = "kuka_lbr_l_tcp";

    vector<string> joint_names = wbc::URDFTools::jointNamesFromURDF(urdf_file);
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(auto n : joint_state.names){
        joint_state[n].position = (double)rand()/RAND_MAX;
        joint_state[n].speed = (double)rand()/RAND_MAX;
        joint_state[n].acceleration = (double)rand()/RAND_MAX;
    }

    cout<<"Input Joint State"<<endl;
    for(auto n : joint_state.names) cout<<joint_state[n].position<<" "; cout<<endl;
    for(auto n : joint_state.names) cout<<joint_state[n].speed<<" "; cout<<endl;
    for(auto n : joint_state.names) cout<<joint_state[n].acceleration<<" "; cout<<endl;

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd((double)rand()/RAND_MAX, Eigen::Vector3d::UnitZ());

    // these are in world coordinates (just to make this test easier) (world based rotated)
    floating_base_state.twist.linear  = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.linear  = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);

    // these are in local coordinates (just to make this test easier)
    floating_base_state.twist.angular = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d((double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX);

    cout<<"Input floating base state"<<endl;
    printRbs(floating_base_state);

    base::samples::RigidBodyStateSE3 rbs_kdl = testRobotModelKDL(urdf_file_with_floating_base, tip_frame, joint_state, floating_base_state);
    base::samples::RigidBodyStateSE3 rbs_rbdl = testRobotModelRBDL(urdf_file, tip_frame, joint_state, floating_base_state);
    base::samples::RigidBodyStateSE3 rbs_hyrodyn = testRobotModelHyrodyn(urdf_file_with_floating_base, sub_mec_file, tip_frame, joint_state, floating_base_state);
    base::samples::RigidBodyStateSE3 rbs_pinocchio = testRobotModelPinocchio(urdf_file, tip_frame, joint_state, floating_base_state);

    cout<<"..................... RobotModelKDL .................."<<endl;
    printRbs(rbs_kdl);
    cout<<"..................... RobotModelRBDL .................."<<endl;
    printRbs(rbs_rbdl);
    cout<<"..................... RobotModelHyrodyn .................."<<endl;
    printRbs(rbs_hyrodyn);
    cout<<"..................... RobotModelPinocchio .................."<<endl;
    printRbs(rbs_pinocchio);

}

