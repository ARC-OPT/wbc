#include "RobotModelHyrodyn.hpp"
#include <base-logging/Logging.hpp>
#include "URDFTools.hpp"

namespace wbc{

RobotModelHyrodyn::RobotModelHyrodyn(){
}

RobotModelHyrodyn::~RobotModelHyrodyn(){
}

bool RobotModelHyrodyn::configure(const std::vector<RobotModelConfig>& model_config){
    clear();

    std::string robot_urdf_file, submechanism_file;
    urdf::ModelInterfaceSharedPtr robot_urdf;
    for(int i = 0; i < model_config.size(); i++){

        const RobotModelConfig& cfg = model_config[i];
        if(cfg.type == ModelType::UNSET){
            LOG_ERROR("Robot Model Config #%i is invalid: Model type unset", i);
            return false;
        }
        if(cfg.file.empty()){
            LOG_ERROR("Robot Model Config #%i is invalid: No model file given in robot model config", i);
            return false;
        }

        switch(cfg.type){
        case URDF: {
            robot_urdf_file = cfg.file;
            if(cfg.floating_base){
                robot_urdf = urdf::parseURDFFile(cfg.file);
                if(!robot_urdf){
                    LOG_ERROR("Unable to parse urdf model from file %s", cfg.file.c_str());
                    return false;
                }
                floating_base_names = URDFTools::addFloatingBaseToURDF(robot_urdf);
                TiXmlDocument *doc = urdf::exportURDF(robot_urdf);
                robot_urdf_file = "/tmp/floating_base_model.urdf";
                doc->SaveFile(robot_urdf_file);
                has_floating_base = true;
            }
            break;
        }
        case SUBMECHANISM:{
            submechanism_file = cfg.file;
            break;
        }
        default:{
            LOG_ERROR_S<<"Robot Model type "<<cfg.type<<" is not supported by Hyrodyn robot model"<<std::endl;
            return false;
        }
        }
    }

    if(robot_urdf_file.empty() || submechanism_file.empty()){
        LOG_ERROR("Robot Model Config is invalid: You have to pass at least one robot model (urdf) file and one submechanism file");
        return false;
    }

    load_robotmodel(robot_urdf_file, submechanism_file);

    base_frame = URDFTools::rootLinkFromURDF(robot_urdf_file);
    for(size_t i = 0; i < jointnames_independent.size(); i++){
        current_joint_state.elements.push_back(base::JointState());
        current_joint_state.names.push_back(jointnames_independent[i]);
    }
    for(size_t i = 0; i < jointnames_active.size(); i++)
        actuated_joint_names.push_back(jointnames_active[i]);

    return true;
}

void RobotModelHyrodyn::update(const base::samples::Joints& joint_state,
                               const base::samples::RigidBodyStateSE3& _floating_base_state){

    RobotModel::update(joint_state, _floating_base_state);

    // Convert floating base state
    uint start_idx = 0;
    if(floating_base_robot){
        y.segment(0,3)   = floating_base_state.pose.position;
        y.segment(3,3)   = floating_base_state.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        yd.segment(0,3)  = floating_base_state.twist.linear;
        yd.segment(3,3)  = floating_base_state.twist.linear;
        ydd.segment(3,3) = floating_base_state.acceleration.linear;
        ydd.segment(3,3) = floating_base_state.acceleration.linear;
        start_idx = 6;
    }
    // Update KDL data types
    for( unsigned int i = start_idx; i < jointnames_independent.size(); ++i){
        const std::string& name =  jointnames_independent[i];
        y[i] = current_joint_state.getElementByName(name).position;
        yd[i] = current_joint_state.getElementByName(name).speed;
        ydd[i] = current_joint_state.getElementByName(name).acceleration;
        Tau_independentjointspace[i] = current_joint_state.getElementByName(name).effort;
    }
}

const base::samples::RigidBodyStateSE3 &RobotModelHyrodyn::rigidBodyState(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != base_frame){
        LOG_ERROR_S<<"Requested Forward kinematics computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    calculate_forward_kinematics(tip_frame);
    rbs.pose.position    = pose.segment(0,3);
    rbs.pose.orientation = base::Quaterniond(pose[6],pose[3],pose[4],pose[5]);
    rbs.twist.linear     = twist.segment(3,3);
    rbs.twist.angular    = twist.segment(0,3);
    rbs.time             = current_joint_state.time;
    rbs.frame_id         = tip_frame;
    return rbs;
}

const base::MatrixXd &RobotModelHyrodyn::jacobian(const std::string &root_frame, const std::string &tip_frame){

    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != base_frame){
        LOG_ERROR_S<<"Requested Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    calculate_space_jacobian(tip_frame);
    uint n_cols = Js.cols();
    jac_map[tip_frame] = base::MatrixXd(6, n_cols);

    jac_map[tip_frame].block(0,0,3,n_cols) = Js.block(3,0,3,n_cols);
    jac_map[tip_frame].block(3,0,3,n_cols) = Js.block(0,0,3,n_cols);
    return jac_map[tip_frame];
}

const base::MatrixXd &RobotModelHyrodyn::jacobianDot(const std::string &root_frame, const std::string &tip_frame){
    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    if(root_frame != base_frame){
        LOG_ERROR_S<<"Requested Jacobian computation for kinematic chain "<<root_frame<<"->"<<tip_frame<<" but hyrodyn robot model always requires the root frame to be the root of the full model"<<std::endl;
        throw std::runtime_error("Invalid root frame");
    }

    calculate_space_jacobian(tip_frame);
    uint n_cols = Js.cols();
    jac_dot_map[tip_frame] = base::MatrixXd(6, n_cols);
    jac_dot_map[tip_frame].setZero();
    return jac_dot_map[tip_frame];
}

const base::MatrixXd &RobotModelHyrodyn::jointSpaceInertiaMatrix(){
    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    calculate_mass_interia_matrix();
    joint_space_inertia_mat = H;
    return joint_space_inertia_mat;
}

const base::VectorXd &RobotModelHyrodyn::biasForces(){
    if(current_joint_state.time.isNull()){
        LOG_ERROR("RobotModelKDL: You have to call update() with appropriately timestamped joint data at least once before requesting kinematic information!");
        throw std::runtime_error(" Invalid call to rigidBodyState()");
    }

    // TODO: Use Hyrodyn ID with zero acceleration here

    VectorXd q(m.dof_count);
    HyRoDyn::calc_sysstate_q(m, *alcs, y, q);

    VectorXd qd(m.dof_count);
    HyRoDyn::calc_sysstate_qdot(m, *alcs, y, yd, qd);

    Math::VectorNd C(m.dof_count);
    NonlinearEffects(m, q, qd, C);
    bias_forces = C;
    return bias_forces;
}

}
