#include "RobotModelHyrodyn.hpp"
#include <urdf_parser/urdf_parser.h>
#include "../../tools/URDFTools.hpp"
#include "../../tools/Logger.hpp"
#include <tinyxml2.h>

namespace wbc{

RobotModelRegistry<RobotModelHyrodyn> RobotModelHyrodyn::reg("hyrodyn");

RobotModelHyrodyn::RobotModelHyrodyn(){
}

RobotModelHyrodyn::~RobotModelHyrodyn(){
}

void RobotModelHyrodyn::clear(){

    RobotModel::clear();

    hyrodyn = hyrodyn::RobotModel_HyRoDyn();
}

void RobotModelHyrodyn::addFloatingBaseToURDF(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::string &world_frame_id){

    std::vector<std::string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                                    "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    auto *doc = urdf::exportURDF(robot_urdf);
    std::string filename = "/tmp/robot_urdf";
    doc->SaveFile(filename.c_str());
    std::ifstream file;
    file.open(filename);
    if(!file){
        std::cout << "Could not open " << filename << std::endl;
        exit(0);
    }
    std::string robot_xml_string( (std::istreambuf_iterator<char>(file) ),(std::istreambuf_iterator<char>()) );
    robot_xml_string.erase(robot_xml_string.find("</robot>"), std::string("</robot>").length());
    std::string floating_base = std::string("  <link name='" + world_frame_id + "'>\n")   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link>" +

            "  <link name='link_floating_base_trans_x'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_trans_y'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_trans_z'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_rot_x'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_rot_y'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

           "  <joint name='floating_base_trans_x' type='prismatic'>\n"   +
             "  <parent link='" + world_frame_id + "'/>\n"   +
             "  <child link='link_floating_base_trans_x'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='1 0 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_trans_y' type='prismatic'>\n"   +
             "  <parent link='link_floating_base_trans_x'/>\n"   +
             "  <child link='link_floating_base_trans_y'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 1 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_trans_z' type='prismatic'>\n"   +
             "  <parent link='link_floating_base_trans_y'/>\n"   +
             "  <child link='link_floating_base_trans_z'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 0 1'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_x' type='revolute'>\n"   +
             "  <parent link='link_floating_base_trans_z'/>\n"   +
             "  <child link='link_floating_base_rot_x'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='1 0 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_y' type='revolute'>\n"   +
             "  <parent link='link_floating_base_rot_x'/>\n"   +
             "  <child link='link_floating_base_rot_y'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 1 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_z' type='revolute'>\n"   +
             "  <parent link='link_floating_base_rot_y'/>\n"   +
             "  <child link='" + robot_urdf->getRoot()->name + "'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 0 1'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n";
    robot_xml_string += floating_base + "</robot>";
    robot_urdf = urdf::parseURDF(robot_xml_string);
}

bool RobotModelHyrodyn::configure(const RobotModelConfig& cfg){

    clear();

    // 1. Load Robot Model

    robot_model_config = cfg;
    robot_urdf = loadRobotURDF(cfg.file_or_string);
    if(!robot_urdf){
        log(logERROR)<<"Unable to parse urdf model";
        return false;
    }
    base_frame = robot_urdf->getRoot()->name;
    URDFTools::applyJointBlacklist(robot_urdf, cfg.joint_blacklist);

    // Add floating base
    has_floating_base = cfg.floating_base;
    world_frame = base_frame;
    if(has_floating_base){
        addFloatingBaseToURDF(robot_urdf);
        world_frame = robot_urdf->getRoot()->name;
    }

    tinyxml2::XMLDocument *doc = urdf::exportURDF(robot_urdf);
    std::string robot_urdf_file = "/tmp/floating_base_model.urdf";
    doc->SaveFile(robot_urdf_file.c_str());
    try{
        hyrodyn.load_robotmodel(robot_urdf_file, cfg.submechanism_file);
    }
    catch(std::exception e){
        log(logERROR) << "Failed to load hyrodyn model from URDF " << robot_urdf_file <<
                       " and submechanism file " << cfg.submechanism_file;
        return false;
    }

    joint_state.resize(hyrodyn.spanningtree_dof);
    joint_names = hyrodyn.jointnames_spanningtree;
    actuated_joint_names = hyrodyn.jointnames_active;
    independent_joint_names = hyrodyn.jointnames_independent;

    // Read Joint Limits
    URDFTools::jointLimitsFromURDF(robot_urdf, joint_limits, actuated_joint_names);

    // 2. Create data structures

    contacts = cfg.contact_points;
    selection_matrix.resize(na(),nj());
    selection_matrix.setZero();
    for(uint i = 0; i < actuated_joint_names.size(); i++)
        selection_matrix(i, nfb() + i) = 1.0;
    joint_weights.resize(na());
    joint_weights.setConstant(1.0);

    log(logDEBUG) << "------------------- WBC RobotModelHyrodyn -----------------";
    log(logDEBUG) << "Robot Name " << robot_urdf->getName();
    log(logDEBUG) << "Floating base robot: " << hyrodyn.floating_base_robot;
    log(logDEBUG) << "Joint Names";
    for(auto n : jointNames())
        log(logDEBUG) << n;
    log(logDEBUG) << "Actuated Joint Names";
    for(auto n : actuatedJointNames())
        log(logDEBUG) << n;
    log(logDEBUG) << "Joint Limits";
    for(uint i = 0; i < na(); i++)
        log(logDEBUG) << actuated_joint_names[i] << ": Max. Pos: " << joint_limits.max.position[i] << ", "
                           << "  Min. Pos: " << joint_limits.min.position[i] << ", "
                           << "  Max. Vel: " << joint_limits.max.velocity[i] << ", "
                           << "  Max. Eff: " << joint_limits.max.effort[i];
    log(logDEBUG) << "------------------------------------------------------------";

    return true;
}

void RobotModelHyrodyn::update(const Eigen::VectorXd& joint_positions,
                               const Eigen::VectorXd& joint_velocities,
                               const Eigen::VectorXd& joint_accelerations,
                               const types::Pose& fb_pose,
                               const types::Twist& fb_twist,
                               const types::SpatialAcceleration& fb_acc){

    assert(configured);
    assert(joint_positions.size() == na());
    assert(joint_velocities.size() == na());
    assert(joint_accelerations.size() == na());

    if(has_floating_base){
        floating_base_state.pose = fb_pose;
        floating_base_state.twist = fb_twist;
        floating_base_state.acceleration = fb_acc;

        // Transformation from fb body linear acceleration to fb joint linear acceleration
        // look at RobotModelRBDL for description
        Eigen::Matrix3d fb_rot = floating_base_state.pose.orientation.toRotationMatrix();
        types::Twist fb_twist = floating_base_state.twist;
        types::SpatialAcceleration fb_acc = floating_base_state.acceleration;

        Eigen::VectorXd spherical_j_vel(6);
        spherical_j_vel << fb_twist.angular, Eigen::Vector3d::Zero();
        Eigen::VectorXd spherical_b_vel(6);
        spherical_b_vel << fb_twist.angular, fb_rot.transpose() * fb_twist.linear;
        Eigen::VectorXd fb_spherical_cross = crossm(spherical_b_vel, spherical_j_vel);
        fb_acc.linear = fb_acc.linear - fb_rot * fb_spherical_cross.tail<3>(); // remove cross contribution from linear acc s(in world coordinates as RBDL want)

        hyrodyn.floating_robot_pose.segment(0,3) = floating_base_state.pose.position;
        hyrodyn.floating_robot_pose[3] = floating_base_state.pose.orientation.x();
        hyrodyn.floating_robot_pose[4] = floating_base_state.pose.orientation.y();
        hyrodyn.floating_robot_pose[5] = floating_base_state.pose.orientation.z();
        hyrodyn.floating_robot_pose[6] = floating_base_state.pose.orientation.w();
        hyrodyn.floating_robot_twist.segment(0,3) = floating_base_state.twist.angular;
        hyrodyn.floating_robot_twist.segment(3,3) = floating_base_state.twist.linear;
        hyrodyn.floating_robot_accn.segment(0,3) = fb_acc.angular;
        hyrodyn.floating_robot_accn.segment(3,3) = fb_acc.linear;

        hyrodyn.y_robot   = joint_positions;
        hyrodyn.yd_robot  = joint_velocities;
        hyrodyn.ydd_robot = joint_accelerations;
            
        hyrodyn.update_all_independent_coordinates();
    }
    else{
        hyrodyn.y_robot   = joint_positions;
        hyrodyn.yd_robot  = joint_velocities;
        hyrodyn.ydd_robot = joint_accelerations;
    }

    // Compute system state
    hyrodyn.calculate_system_state();

    for(size_t i = 0; i < hyrodyn.jointnames_spanningtree.size(); i++){
        const std::string &name = hyrodyn.jointnames_spanningtree[i];
        joint_state.position     = hyrodyn.Q;
        joint_state.velocity     = hyrodyn.QDot;
        joint_state.acceleration = hyrodyn.QDDot;
    }
}

const types::Pose &RobotModelHyrodyn::pose(const std::string &frame_id){
    assert(updated);
    assert(frame_id != world_frame);

    if(pose_map.count(frame_id) == 0)
        pose_map[frame_id] = Pose();

    if(pose_map[frame_id].needs_recompute){
        hyrodyn.calculate_forward_kinematics(frame_id);    
        pose_map[frame_id].data.position    = hyrodyn.pose.segment(0,3);
        pose_map[frame_id].data.orientation = Eigen::Quaterniond(hyrodyn.pose[6],hyrodyn.pose[3],hyrodyn.pose[4],hyrodyn.pose[5]);
        pose_map[frame_id].needs_recompute = false;
    }
    return pose_map[frame_id].data;
}

const types::Twist &RobotModelHyrodyn::twist(const std::string &frame_id){
    assert(updated);
    assert(frame_id != world_frame);

    if(twist_map.count(frame_id) == 0)
        twist_map[frame_id] = Twist();

    if(twist_map[frame_id].needs_recompute){
        hyrodyn.calculate_forward_kinematics(frame_id);    
        twist_map[frame_id].data.linear  = hyrodyn.twist.segment(3,3);
        twist_map[frame_id].data.angular = hyrodyn.twist.segment(0,3);
        twist_map[frame_id].needs_recompute = false;
    }
    return twist_map[frame_id].data;
}

const types::SpatialAcceleration &RobotModelHyrodyn::acceleration(const std::string &frame_id){
    assert(updated);
    assert(frame_id != world_frame);

    if(acc_map.count(frame_id) == 0)
        acc_map[frame_id] = SpatialAcceleration();

    if(acc_map[frame_id].needs_recompute){
        hyrodyn.calculate_forward_kinematics(frame_id);    
        acc_map[frame_id].data.linear  = hyrodyn.spatial_acceleration.segment(3,3);
        acc_map[frame_id].data.angular = hyrodyn.spatial_acceleration.segment(0,3);
        acc_map[frame_id].needs_recompute = false;
    }
    return acc_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelHyrodyn::spaceJacobian(const std::string &frame_id){
    assert(updated);
    assert(frame_id != world_frame);

    if(space_jac_map.count(frame_id) == 0)
        space_jac_map[frame_id] = Matrix();

    if(space_jac_map[frame_id].needs_recompute){
        
        space_jac_map[frame_id].data.resize(6,na()+nfb());
        space_jac_map[frame_id].data.setZero();
        if(hyrodyn.floating_base_robot){
            hyrodyn.calculate_space_jacobian_actuation_space_including_floatingbase(frame_id);
            uint n_cols = hyrodyn.Jsufb.cols();
            space_jac_map[frame_id].data.block(0,0,3,n_cols) = hyrodyn.Jsufb.block(3,0,3,n_cols);
            space_jac_map[frame_id].data.block(3,0,3,n_cols) = hyrodyn.Jsufb.block(0,0,3,n_cols);
        }else{
            hyrodyn.calculate_space_jacobian_actuation_space(frame_id);
            uint n_cols = hyrodyn.Jsu.cols();
            space_jac_map[frame_id].data.block(0,0,3,n_cols) = hyrodyn.Jsu.block(3,0,3,n_cols);
            space_jac_map[frame_id].data.block(3,0,3,n_cols) = hyrodyn.Jsu.block(0,0,3,n_cols);
        }
        space_jac_map[frame_id].needs_recompute = false;

    }

    return space_jac_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelHyrodyn::bodyJacobian(const std::string &frame_id){
    assert(updated);
    assert(frame_id != world_frame);

    if(body_jac_map.count(frame_id) == 0)
        body_jac_map[frame_id] = Matrix();

    if(body_jac_map[frame_id].needs_recompute){
        
        body_jac_map[frame_id].data.resize(6,na()+nfb());
        body_jac_map[frame_id].data.setZero();
        if(hyrodyn.floating_base_robot){
            hyrodyn.calculate_body_jacobian_actuation_space_including_floatingbase(frame_id);
            uint n_cols = hyrodyn.Jsufb.cols();
            body_jac_map[frame_id].data.block(0,0,3,n_cols) = hyrodyn.Jbufb.block(3,0,3,n_cols);
            body_jac_map[frame_id].data.block(3,0,3,n_cols) = hyrodyn.Jbufb.block(0,0,3,n_cols);
        }else{
            hyrodyn.calculate_body_jacobian_actuation_space(frame_id);
            uint n_cols = hyrodyn.Jsu.cols();
            body_jac_map[frame_id].data.block(0,0,3,n_cols) = hyrodyn.Jbu.block(3,0,3,n_cols);
            body_jac_map[frame_id].data.block(3,0,3,n_cols) = hyrodyn.Jbu.block(0,0,3,n_cols);
        }
        body_jac_map[frame_id].needs_recompute = false;
    }

    return body_jac_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelHyrodyn::comJacobian(){
    assert(updated);

    if(com_jac.empty())
        com_jac.push_back(Matrix());

    if(com_jac[0].needs_recompute){

        hyrodyn.calculate_com_jacobian();
        com_jac[0].data.resize(3,na()+nfb());
        com_jac[0].data = hyrodyn.Jcom;
        com_jac[0].needs_recompute = false;
    }

    return com_jac[0].data;
}

const types::SpatialAcceleration &RobotModelHyrodyn::spatialAccelerationBias(const std::string &frame_id){

    assert(updated);
    assert(frame_id != world_frame);

    if(spatial_acc_bias_map.count(frame_id) == 0)
        spatial_acc_bias_map[frame_id] = SpatialAcceleration();

    if(spatial_acc_bias_map[frame_id].needs_recompute){

        hyrodyn.calculate_spatial_acceleration_bias(frame_id);

        spatial_acc_bias_map[frame_id].data.linear = hyrodyn.spatial_acceleration_bias.segment(3,3);
        spatial_acc_bias_map[frame_id].data.angular = hyrodyn.spatial_acceleration_bias.segment(0,3);
        spatial_acc_bias_map[frame_id].needs_recompute = false;
    }

    return spatial_acc_bias_map[frame_id].data;
}

const Eigen::MatrixXd &RobotModelHyrodyn::jointSpaceInertiaMatrix(){

    assert(updated);

    if(joint_space_inertia_mat.empty())
        joint_space_inertia_mat.push_back(Matrix());

    if(joint_space_inertia_mat[0].needs_recompute){
        if(hyrodyn.floating_base_robot){
            hyrodyn.calculate_mass_interia_matrix_actuation_space_including_floatingbase();
            joint_space_inertia_mat[0].data = hyrodyn.Hufb;
        }
        else{
            hyrodyn.calculate_mass_interia_matrix_actuation_space();
            joint_space_inertia_mat[0].data = hyrodyn.Hu;
        }
        joint_space_inertia_mat[0].needs_recompute = false;
    }

    return joint_space_inertia_mat[0].data;
}

const Eigen::VectorXd &RobotModelHyrodyn::biasForces(){

    assert(updated);

    if(bias_forces.empty())
        bias_forces.push_back(Vector());

    if(bias_forces[0].needs_recompute){
        hyrodyn.ydd.setZero();
        if(hyrodyn.floating_base_robot){
            hyrodyn.calculate_inverse_dynamics_including_floatingbase();
            bias_forces[0].data = hyrodyn.Tau_actuated_floatingbase;
        }
        else{
            hyrodyn.calculate_inverse_dynamics();
            bias_forces[0].data = hyrodyn.Tau_actuated;
        }
        bias_forces[0].needs_recompute = false;
    }

    return bias_forces[0].data;
}

const types::RigidBodyState& RobotModelHyrodyn::centerOfMass(){

    assert(updated);

    if(com_rbs.empty())
        com_rbs.push_back(RigidBodyState());

    if(com_rbs[0].needs_recompute){

        hyrodyn.calculate_com_properties();

        com_rbs[0].data.pose.position = hyrodyn.com;
        com_rbs[0].data.pose.orientation.setIdentity();
        com_rbs[0].data.twist.linear = hyrodyn.com_vel;
        com_rbs[0].data.twist.angular.setZero();
        com_rbs[0].data.acceleration.linear = hyrodyn.com_acc; // TODO: double check CoM acceleration
        com_rbs[0].data.acceleration.angular.setZero();
        com_rbs[0].needs_recompute = false;
    }

    return com_rbs[0].data;
}

const Eigen::VectorXd& RobotModelHyrodyn::inverseDynamics(const Eigen::VectorXd& qdd_ref, const std::vector<types::Wrench>& f_ext){

    assert(updated);

    if(qdd_ref.size() == 0)
        hyrodyn.udd = qdd;
    else
        hyrodyn.udd = qdd_ref;
    hyrodyn.u = q;
    hyrodyn.ud = qd;

    hyrodyn.calculate_forward_system_state();

    uint nc = robot_model_config.contact_points.size();
    hyrodyn.wrench_interaction.resize(nc);
    hyrodyn.wrench_points.resize(nc);
    hyrodyn.wrench_resolution.resize(nc);
    hyrodyn.f_ext.resize(nc);
    for(uint i = 0; i < nc; i++){
        hyrodyn.wrench_points[i] = robot_model_config.contact_points[i].frame_id;
        hyrodyn.wrench_interaction[i] = true; // Resistive
        hyrodyn.wrench_resolution[i]  = true; // body coordinates
        hyrodyn.f_ext[i].segment(0,3) = f_ext[i].torque;
        hyrodyn.f_ext[i].segment(3,3) = f_ext[i].force;        
    }
    hyrodyn.calculate_inverse_dynamics();
    hyrodyn.calculate_inverse_statics();
    tau = hyrodyn.Tau_actuated + hyrodyn.Tau_actuated_ext;
    return tau;
}

}
