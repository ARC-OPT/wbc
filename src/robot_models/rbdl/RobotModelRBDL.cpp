#include "RobotModelRBDL.hpp"
#include "tools/URDFTools.hpp"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <stack>
#include <base-logging/Logging.hpp>
#include <sstream>
#include <fstream>

namespace wbc {

RobotModelRBDL::RobotModelRBDL(){

}

RobotModelRBDL::~RobotModelRBDL(){

}

std::vector<std::string> RobotModelRBDL::jointNamesInRBDLOrder(const std::string &urdf_file){
    urdf::ModelInterfaceSharedPtr robot_urdf = urdf::parseURDFFile(urdf_file);
    if(!robot_urdf){
        LOG_ERROR_S << "Unable to parse urdf from file " << urdf_file <<std::endl;
        throw std::runtime_error("Failed to parse urdf file");
    }
    std::vector<std::string> joint_names;
    std::map<std::string, urdf::LinkSharedPtr> link_map = robot_urdf->links_;
    std::stack< std::shared_ptr<urdf::Link> > link_stack;
    link_stack.push (link_map[(robot_urdf->getRoot()->name)]);
    std::stack<int> joint_index_stack;
    joint_index_stack.push(0);
    while (link_stack.size() > 0) {
        std::shared_ptr<urdf::Link> cur_link = link_stack.top();
        unsigned int joint_idx = joint_index_stack.top();
        if (joint_idx < cur_link->child_joints.size()) {
            std::shared_ptr<urdf::Joint> cur_joint = cur_link->child_joints[joint_idx];
            joint_index_stack.pop();
            joint_index_stack.push (joint_idx + 1);
            link_stack.push (link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);
            if(cur_joint->type != urdf::Joint::FIXED)
                joint_names.push_back(cur_joint->name);
        }
        else{
            link_stack.pop();
            joint_index_stack.pop();
        }
    }
    return joint_names;
}

bool RobotModelRBDL::configure(const RobotModelConfig& cfg){

    rbdl_model.reset();
    rbdl_model = std::make_shared<RigidBodyDynamics::Model>();

    if(!RigidBodyDynamics::Addons::URDFReadFromFile(cfg.file.c_str(), rbdl_model.get(), cfg.floating_base)){
        LOG_ERROR_S << "Unable to parse urdf from file " << cfg.file << std::endl;
        return false;
    }

    URDFTools::jointLimitsFromURDF(cfg.file, joint_limits);

    joint_names = jointNamesInRBDLOrder(cfg.file);
    independent_joint_names = actuated_joint_names = joint_names;

    joint_state.names = joint_names;
    joint_state.elements.resize(joint_names.size());


    // Fixed base: If the robot has N dof, q_size = qd_size = N
    // Floating base: If the robot has N dof, q_size = N+7, qd_size = N+6
    q.resize(rbdl_model->q_size);
    qd.resize(rbdl_model->qd_size);
    qdd.resize(rbdl_model->qd_size);

    std::cout<<"Dof count: "<<rbdl_model->dof_count<<std::endl;
    std::cout<<"q_size: "<<rbdl_model->q_size<<std::endl;
    std::cout<<"qd_size: "<<rbdl_model->qdot_size<<std::endl;

    world_frame = cfg.world_frame_id;
    base_frame = URDFTools::rootLinkFromURDF(cfg.file);
    has_floating_base = cfg.floating_base;

    for(auto n : joint_names)
        std::cout<< n << std::endl;

    return true;
}


void RobotModelRBDL::updateFloatingBase(const base::samples::RigidBodyStateSE3& floating_base_state_in){
    floating_base_state = floating_base_state_in;

    // Transformation from fb body linear acceleration to fb joint linear acceleration:
    // since RBDL treats the floating base as XYZ translation followed by spherical
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

    Eigen::VectorXd fb_spherical_cross = RigidBodyDynamics::Math::crossm(spherical_b_vel, spherical_j_vel);
    // remove cross contribution from linear acc s(in world coordinates as RBDL want)
    fb_acc.linear = fb_acc.linear - fb_rot * fb_spherical_cross.tail<3>();

    int floating_body_id = rbdl_model->GetBodyId(base_frame.c_str());
    rbdl_model->SetQuaternion(floating_body_id, RigidBodyDynamics::Math::Quaternion(floating_base_state_in.pose.orientation.coeffs()), q);

    for(int i = 0; i < 3; i++){
        q[i] = floating_base_state.pose.position[i];
        qd[i] = fb_twist.linear[i];
        qdd[i] = fb_acc.linear[i];
        qd[i+3] = fb_twist.angular[i];
        qdd[i+3] = fb_acc.angular[i];
    }
}

void RobotModelRBDL::update(const base::samples::Joints& joint_state_in,
                            const base::samples::RigidBodyStateSE3& floating_base_state_in){

    if(joint_state_in.elements.size() != joint_state_in.names.size()){
        LOG_ERROR_S << "Size of names and size of elements in joint state do not match"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    if(joint_state_in.time.isNull()){
        LOG_ERROR_S << "Joint State does not have a valid timestamp. Or do we have 1970?"<<std::endl;
        throw std::runtime_error("Invalid joint state");
    }

    uint start_idx = 0;
    if(has_floating_base){
        start_idx = 6;
        updateFloatingBase(floating_base_state_in);
    }

    for(int i = 0; i < joint_names.size(); i++){
        const std::string &name = joint_names[i];
        try{
            const base::JointState& state = joint_state[name];
            q[i+start_idx] = state.position;
            qd[i+start_idx] = state.speed;
            qdd[i+start_idx] = state.acceleration;
        }
        catch(base::samples::Joints::InvalidName e){
            LOG_ERROR_S << "Joint " << name << " is a non-fixed joint in robot model, but it is not given in joint state vector" << std::endl;
            throw e;
        }
    }
}

const base::samples::RigidBodyStateSE3 &RobotModelRBDL::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

}

const base::MatrixXd &RobotModelRBDL::spaceJacobian(const std::string &root_frame, const std::string &tip_frame){

}

const base::MatrixXd &RobotModelRBDL::bodyJacobian(const std::string &root_frame, const std::string &tip_frame){

}

const base::MatrixXd &RobotModelRBDL::comJacobian(){

}

const base::MatrixXd &RobotModelRBDL::jacobianDot(const std::string &root_frame, const std::string &tip_frame){

}

const base::Acceleration &RobotModelRBDL::spatialAccelerationBias(const std::string &root_frame, const std::string &tip_frame){

}

const base::MatrixXd &RobotModelRBDL::jointSpaceInertiaMatrix(){

}

const base::VectorXd &RobotModelRBDL::biasForces(){

}

const base::samples::RigidBodyStateSE3& RobotModelRBDL::centerOfMass(){

}

void RobotModelRBDL::computeInverseDynamics(base::commands::Joints &solver_output){

}

}
