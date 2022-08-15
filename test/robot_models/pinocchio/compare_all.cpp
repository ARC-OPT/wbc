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

void printRbs(base::samples::RigidBodyStateSE3 rbs){
    cout<<"Position:      "<<rbs.pose.position.transpose()<<endl;
    cout<<"Orientation:   "<<rbs.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist linear:  "<<rbs.twist.linear.transpose()<<endl;
    cout<<"Twist angular: "<<rbs.twist.angular.transpose()<<endl;
    cout<<"Acc linear:    "<<rbs.acceleration.linear.transpose()<<endl;
    cout<<"Acc angular:   "<<rbs.acceleration.angular.transpose()<<endl<<endl;
}

base::samples::RigidBodyStateSE3 testRobotModelKDL(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state){
    string root_frame = wbc::URDFTools::rootLinkFromURDF(urdf_file);

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(urdf_file,tree))
        abort();

    uint nj = joint_state.size();

    // Setup solvers
    KDL::Chain chain;
    tree.getChain(root_frame,tip_frame,chain);
    KDL::ChainFkSolverVel_recursive solver(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);
    KDL::ChainJntToJacDotSolver jac_dot_solver(chain);
    jac_dot_solver.setRepresentation(KDL::ChainJntToJacDotSolver::HYBRID);

    // Setup KDL state vectors
    KDL::JntArrayVel q_and_qd_kdl(nj);
    KDL::JntArrayAcc qdd_kdl(nj);
    for(int i = 0; i < nj; i++){
        q_and_qd_kdl.q(i) = joint_state[i].position;
        q_and_qd_kdl.qdot(i) = qdd_kdl.qdot(i) = joint_state[i].speed;
        qdd_kdl.qdotdot(i) = joint_state[i].acceleration;
    }

    // Compute pose & twist
    KDL::FrameVel frame_vel;
    solver.JntToCart(q_and_qd_kdl,frame_vel);
    KDL::Frame pose_kdl = frame_vel.GetFrame();
    KDL::Twist twist_kdl = frame_vel.GetTwist();

    // Compute acc
    KDL::Jacobian jac_kdl(nj);
    if(jac_solver.JntToJac(q_and_qd_kdl.q, jac_kdl))
        abort();
    KDL::Jacobian jac_dot_kdl(nj);
    if(jac_dot_solver.JntToJacDot(q_and_qd_kdl, jac_dot_kdl))
        abort();
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

base::samples::RigidBodyStateSE3 testRobotModelRBDL(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state){

    RigidBodyDynamics::Model rbdl_model;
    if(!Addons::URDFReadFromFile(urdf_file.c_str(), &rbdl_model, false))
        abort();

    // Setup RBDL state vectors
    uint nj = joint_state.size();
    Eigen::VectorXd q(nj),qd(nj),qdd(nj);
    for(int i = 0; i < nj; i++){
        q[i] = joint_state[i].position;
        qd[i] = joint_state[i].speed;
        qdd[i] = joint_state[i].acceleration;
    }

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

    return rbs;
}


base::samples::RigidBodyStateSE3 testRobotModelHyrodyn(const string &urdf_file, const string &sub_mec_file, const string &tip_frame, const base::samples::Joints &joint_state){
    hyrodyn::RobotModel_HyRoDyn hyrodyn;
    hyrodyn.load_robotmodel(urdf_file, sub_mec_file);

    uint nj = joint_state.size();
    for( unsigned int i = 0; i < nj; ++i){
        hyrodyn.y_robot[i]   = joint_state[i].position;
        hyrodyn.yd_robot[i]  = joint_state[i].speed;
        hyrodyn.ydd_robot[i] = joint_state[i].acceleration;
    }
    hyrodyn.update_all_independent_coordinates();


    base::samples::RigidBodyStateSE3 rbs;
    hyrodyn.calculate_forward_kinematics(tip_frame);
    rbs.pose.position        = hyrodyn.pose.segment(0,3);
    rbs.pose.orientation     = base::Quaterniond(hyrodyn.pose[6],hyrodyn.pose[3],hyrodyn.pose[4],hyrodyn.pose[5]);
    rbs.twist.linear         = hyrodyn.twist.segment(3,3);
    rbs.twist.angular        = hyrodyn.twist.segment(0,3);
    rbs.acceleration.linear  = hyrodyn.spatial_acceleration.segment(3,3);
    rbs.acceleration.angular = hyrodyn.spatial_acceleration.segment(0,3);
    return rbs;
}


base::samples::RigidBodyStateSE3 testRobotModelPinocchio(const string &urdf_file, const string &tip_frame, const base::samples::Joints &joint_state){
    urdf::ModelInterfaceSharedPtr robot_urdf = urdf::parseURDFFile(urdf_file);
    if(!robot_urdf)
        abort();
    pinocchio::Model model;
    pinocchio::urdf::buildModel(robot_urdf,model);

    // Setup Pinocchio state vectors
    uint nj = joint_state.size();
    Eigen::VectorXd q(nj),qd(nj),qdd(nj);
    for(int i = 0; i < nj; i++){
        q[i] = joint_state[i].position;
        qd[i] = joint_state[i].speed;
        qdd[i] = joint_state[i].acceleration;
    }

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
    return rbs;
}

int main(){

    srand(time(NULL));

    string urdf_file = "../../../../models/kuka/urdf/kuka_iiwa.urdf";
    string sub_mec_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
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

    base::samples::RigidBodyStateSE3 rbs_kdl = testRobotModelKDL(urdf_file, tip_frame, joint_state);
    base::samples::RigidBodyStateSE3 rbs_rbdl = testRobotModelRBDL(urdf_file, tip_frame, joint_state);
    base::samples::RigidBodyStateSE3 rbs_hyrodyn = testRobotModelHyrodyn(urdf_file, sub_mec_file, tip_frame, joint_state);
    base::samples::RigidBodyStateSE3 rbs_pinocchio = testRobotModelPinocchio(urdf_file, tip_frame, joint_state);

    cout<<"..................... RobotModelKDL .................."<<endl;
    printRbs(rbs_kdl);
    cout<<"..................... RobotModelRBDL .................."<<endl;
    printRbs(rbs_rbdl);
    cout<<"..................... RobotModelHyrodyn .................."<<endl;
    printRbs(rbs_hyrodyn);
    cout<<"..................... RobotModelPinocchio .................."<<endl;
    printRbs(rbs_pinocchio);

}
