#include "WbcVelocity.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/Logging.hpp>
#include "TaskFrameKDL.hpp"
#include "OptProblem.hpp"
#include <kdl_conversions/KDLConversions.hpp>

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity() : Wbc(),
    configured(false){
}

WbcVelocity::~WbcVelocity(){
    clear();
}

void WbcVelocity::clear(){

    for(uint prio = 0; prio < constraints.size(); prio++ ){
        for(uint j = 0; j < constraints[prio].size(); j++)
            delete constraints[prio][j];
        constraints[prio].clear();
    }

    constraints.clear();
    joint_index_map.clear();
    configured = false;
}

bool WbcVelocity::configure(const std::vector<ConstraintConfig> &config,
                            const std::vector<std::string> &joint_names){

    // Erase constraints, jacobians and joint indices
    clear();

    if(config.empty()){
        LOG_ERROR("Constraint Configuration is empty!");
        return false;
    }
    if(joint_names.empty()){
        LOG_ERROR("Joint name vector is empty");
        return false;
    }

    // Create joint index map. This defines the order of the joints in the task Jacobians computed here. Note
    // that the order of joints in the task frames can be different.
    for(uint i = 0; i < joint_names.size(); i++)
        joint_index_map[joint_names[i]] = i;

    std::vector< std::vector<ConstraintConfig> > sorted_config;
    sortConfigByPriority(config, sorted_config);

    constraints.resize(sorted_config.size());
    for(size_t i = 0; i < sorted_config.size(); i++){

        constraints[i].resize(sorted_config[i].size());

        for(size_t j = 0; j < sorted_config[i].size(); j++)
            constraints[i][j] = new KinematicConstraintKDL(sorted_config[i][j], joint_names.size());
    }

    LOG_DEBUG("WBC Configuration: \n")

    LOG_DEBUG("Joint Indices: \n");
    for(JointIndexMap::iterator it = joint_index_map.begin(); it != joint_index_map.end(); it++)
        LOG_DEBUG_S<<it->first.c_str()<<": "<<it->second;

    LOG_DEBUG("\nTask Frames: \n");
    std::vector<std::string> tf_ids = getTaskFrameIDs(config);
    for(size_t i = 0; i < tf_ids.size(); i++)
        LOG_DEBUG_S<<tf_ids[i];

    LOG_DEBUG("\nConstraints: \n");
    for(size_t i = 0; i < constraints.size(); i++){
        for(size_t j = 0; j < constraints[i].size(); j++)
            LOG_DEBUG_S<<constraints[i][j]->config.name.c_str()<<": prio: "<<constraints[i][j]->config.priority<<", type: "<<constraints[i][j]->config.type;
    }
    LOG_DEBUG("...............................\n")

    configured = true;

    return true;
}

void WbcVelocity::setupOptProblem(const std::vector<TaskFrame*> &task_frames, OptProblem &opt_problem)
{
    if(!configured)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    HierarchicalWeightedLS& opt_problem_ls = (HierarchicalWeightedLS& )opt_problem;

    if(opt_problem_ls.prios.size() != constraints.size())
        opt_problem_ls.prios.resize(constraints.size());

    // Walk through all priorities and update optimzation problem. The outcome will be
    //    A - Vector of constraint matrices. One matrix for each priority
    //    y - Vector of constraint velocities. One vector for each priority
    //    W - Vector of constraint weights. One vector for each priority
    //
    // These entities will form linear constraints W*A * x = y for the optimization problem, which can be solved by
    // generalized matrix inversion
    for(uint prio = 0; prio < constraints.size(); prio++)
    {
        uint n_vars_prio = getConstraintVariablesPerPrio()[prio]; //Number of constraint variables on the whole priority
        opt_problem_ls.prios[prio].resize(n_vars_prio, joint_index_map.size());

        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraints[prio].size(); i++)
        {
            KinematicConstraintKDL *constraint = (KinematicConstraintKDL*)constraints[prio][i];
            const uint n_vars = constraint->no_variables;

            //Check task timeout
            if(constraint->config.timeout > 0){
                double diff = (base::Time::now() - constraint->time).toSeconds();

                if(diff > constraint->config.timeout)
                    constraint->constraint_timed_out = 1;
                else
                    constraint->constraint_timed_out = 0;
            }
            else
                constraint->constraint_timed_out = 0;

            constraint->weights_root = constraint->weights;

            if(constraint->config.type == cart)
            {
                const std::string &tf_root_name = constraint->config.root;
                const std::string &tf_tip_name = constraint->config.tip;
                const std::string &tf_ref_name = constraint->config.ref_frame;

                TaskFrameKDL* tf_root = (TaskFrameKDL*)getTaskFrameByName(task_frames, tf_root_name);
                TaskFrameKDL* tf_tip = (TaskFrameKDL*)getTaskFrameByName(task_frames, tf_tip_name);
                TaskFrameKDL* tf_ref_frame = (TaskFrameKDL*)getTaskFrameByName(task_frames, tf_ref_name);

                //Create constraint jacobian
                constraint->pose_tip_in_root = tf_root->pose_kdl.Inverse() * tf_tip->pose_kdl;
                constraint->jac_helper.data.setIdentity();
                constraint->jac_helper.changeRefPoint(-constraint->pose_tip_in_root.p);
                constraint->jac_helper.changeRefFrame(tf_root->pose_kdl);

                //Invert constraint Jacobian
                Eigen::VectorXd tmp(6); // helper var. Not used here.
                KDL::svd_eigen_HH(constraint->jac_helper.data, constraint->Uf, constraint->Sf, constraint->Vf, tmp);

                for (unsigned int j = 0; j < constraint->Sf.size(); j++)
                {
                    if (constraint->Sf(j) > 0)
                        constraint->Uf.col(j) *= 1 / constraint->Sf(j);
                    else
                        constraint->Uf.col(j).setZero();
                }
                constraint->H = (constraint->Vf * constraint->Uf.transpose());

                const KDL::Jacobian& jac_root =  tf_root->full_robot_jacobian;
                const KDL::Jacobian& jac_tip =  tf_tip->full_robot_jacobian;

                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root:
                constraint->A = constraint->H.block(0, 0, n_vars, 6) * jac_tip.data
                        -(constraint->H.block(0, 0, n_vars, 6) * jac_root.data);

                //Convert input twist from ref_frame to root of the kinematic chain
                //IMPORTANT: In KDL there are two ways to transform a twist:
                //    - KDL::Frame*KDL::Twist transforms both the reference point in which the twist is expressed AND the reference frame
                //    - KDL::Rotation*KDL::Twist transform only the reference frame!
                //    - We use KDL::Rotation*KDL::Twist here!
                //    - The difference is that with second method, after transformation, the rotational components of the twist will act like a rotation around the origin
                //      of ref_frame, expressed in the root frame of the robot. With the first method the ref_frame would rotate around the root
                //      frame (e.g. like a satellite rotates around earth), which means that rotational components, would produce translational
                //      velocities after transformation to root frame. If the twist has only translational components there is no difference between
                //      the two methods
                constraint->pose_ref_frame_in_root = tf_root->pose_kdl.Inverse() * tf_ref_frame->pose_kdl;
                KDL::Twist tmp_twist;
                for(uint i = 0; i < 6; i++)
                    tmp_twist(i) = constraint->y_ref(i);
                tmp_twist = constraint->pose_ref_frame_in_root.M * tmp_twist;
                for(uint i = 0; i < 6; i++)
                    constraint->y_ref_root(i) = tmp_twist(i);

                // Also convert the weight vector to the root frame. Weights are, as all other inputs, defined in ref_frame.
                for(uint i = 0; i < 6; i++)
                    tmp_twist(i) = constraint->weights(i);
                tmp_twist = constraint->pose_ref_frame_in_root.M * tmp_twist;
                for(uint i = 0; i < 6; i++)
                    constraint->weights_root(i) = fabs(tmp_twist(i)); //Take absolute value of weight, since weights must be positive
            }
            else{
                for(uint k = 0; k < constraint->config.joint_names.size(); k++){

                    //Joint space constraints: constraint matrix has only ones and Zeros
                    //IMPORTANT: The joint order in the constraints might be different than in wbc.
                    //Thus, for joint space constraints, the joint indices have to be mapped correctly.
                    const std::string &joint_name = constraint->config.joint_names[k];
                    if(joint_index_map.count(joint_name) == 0)
                    {
                        LOG_ERROR("Constraint %s contains joint %s, but this joint has not been configured in joint names", constraint->config.name.c_str(), joint_name.c_str());
                        throw std::invalid_argument("Invalid Constraint config");
                    }
                    uint idx = joint_index_map[joint_name];
                    constraint->A(k,idx) = 1.0;
                    constraint->y_ref_root = constraint->y_ref; // In joint space, y_ref is of yourse equal to y_ref_root
                }
            }

            constraint->time = base::Time::now();

            // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
            // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
            if(constraint->activation == 0){
                constraint->y_ref.setZero();
                constraint->y_ref_root.setZero();
            }

            // Insert constraints into equation system of current priority at the correct position
            opt_problem_ls.prios[prio].W.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->constraint_timed_out);
            opt_problem_ls.prios[prio].A.block(row_index, 0, n_vars, joint_index_map.size()) = constraint->A;
            opt_problem_ls.prios[prio].y_ref.segment(row_index, n_vars) = constraint->y_ref_root;

            row_index += n_vars;
        }
    }
}

void WbcVelocity::setReference(const std::string &name,
                               const base::commands::Joints& reference){

    Constraint* constraint = getConstraint(name);

    if(constraint->config.type != jnt){
        LOG_ERROR("Reference input of constraint %s is in joint space but constraint is Cartesian", constraint->config.name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(reference.size() != constraint->no_variables){
        LOG_ERROR("Size for joint reference of constraint %s should be %i but is %i",
                  name.c_str(), constraint->no_variables, reference.size());
        throw std::invalid_argument("Invalid reference input");
    }

    constraint->time = reference.time; // This value will also be used for checking the constraint timeout!

    for(uint i = 0; i < constraint->config.joint_names.size(); i++){
        uint idx;
        try{
            idx = reference.mapNameToIndex(constraint->config.joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Constraint with name %s expects joint %s, but this joint is not in reference vector!",
                      name.c_str(), constraint->config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid reference input");
        }

        if(!reference[idx].hasSpeed()){
            LOG_ERROR("Reference input for joint %s of constraint %s has no speed value",
                      constraint->config.joint_names[i].c_str(), name.c_str());
            throw std::invalid_argument("Invalid reference input");
        }

        constraint->y_ref(i) = reference[idx].speed;
    }
}

void WbcVelocity::setReference(const std::string &name,
                               const base::samples::RigidBodyState& reference){

    Constraint* constraint = getConstraint(name);

    if(constraint->config.type != cart){
        LOG_ERROR("Reference input of constraint %s is Cartesian but constraint is in joint space", name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }
    if(!reference.hasValidVelocity() ||
       !reference.hasValidAngularVelocity()){
        LOG_ERROR("Reference input of constraint %s has invalid velocity and/or angular velocity", name.c_str());
        throw std::invalid_argument("Invalid reference input");
    }

    constraint->time = reference.time;

    constraint->y_ref.segment(0,3) = reference.velocity;
    constraint->y_ref.segment(3,3) = reference.angular_velocity;
}

void WbcVelocity::evaluateConstraints(const base::VectorXd& solver_output,
                                      const base::VectorXd& robot_vel){

    for(uint prio = 0; prio < constraints.size(); prio++)
    {
        for(uint i = 0; i < constraints[prio].size(); i++){

            KinematicConstraintKDL* constraint = (KinematicConstraintKDL*)constraints[prio][i];

            constraint->y_solution = constraint->A * solver_output;
            constraint->y = constraint->A * robot_vel;
        }
    }
}
}
