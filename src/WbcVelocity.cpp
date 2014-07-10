#include "WbcVelocity.hpp"
#include "ExtendedConstraint.hpp"
#include "TaskFrame.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity() :
    configured_(false),
    temp_(Eigen::VectorXd(6)),
    no_robot_joints_(0){
}

WbcVelocity::~WbcVelocity(){
    clear();
}

void WbcVelocity::clear(){

    for(uint prio = 0; prio < constraint_vector_.size(); prio++ ){
        for(uint j = 0; j < constraint_vector_[prio].size(); j++)
            delete constraint_vector_[prio][j];
        constraint_vector_[prio].clear();
    }

    tf_map_.clear();
    constraint_map_.clear();
    constraint_vector_.clear();
    joint_index_map_.clear();
    solver_input_.clear();
    no_robot_joints_ = 0;
}

bool WbcVelocity::configure(const std::vector<ConstraintConfig> &config,
                            const std::vector<std::string> &joint_names,
                            bool constraints_active,
                            double constraints_timeout){

    clear();

    constraint_timeout_ = constraints_timeout;
    has_timeout_ = true;
    if(base::isUnset(constraints_timeout))
        has_timeout_ = false;

    no_robot_joints_ = joint_names.size();

    // Create joint index map
    joint_index_map_.clear();
        for(uint i = 0; i < joint_names.size(); i++)
            joint_index_map_[joint_names[i]] = i;

    //
    // Create Constraints and sort them by priority
    //
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++)
    {
        if(config[i].priority < 0)
        {
            LOG_ERROR("Constraint Priorities must be >= 0. Constraint priority of constraint %s is %i", config[i].name.c_str(), config[i].priority);
            return false;
        }
        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    constraint_vector_.resize(max_prio + 1);
    for(uint i = 0; i < config.size(); i++)
    {
        ExtendedConstraint* constraint = new ExtendedConstraint(config[i], no_robot_joints_, constraints_active);
        constraint_vector_[config[i].priority].push_back(constraint);

        //Also put Constraints in a map that associates them with their names (for easier access)
        if(constraint_map_.count(config[i].name) != 0)
        {
            LOG_ERROR("Constraint with name %s already exists! Constraint names must be unique", config[i].name.c_str());
            return false;
        }
        constraint_map_[config[i].name] = constraint;
    }

    //Erase empty priorities
    for(uint prio = 0; prio < constraint_vector_.size(); prio++)
    {
        if(constraint_vector_[prio].empty())
        {
            constraint_vector_.erase(constraint_vector_.begin() + prio, constraint_vector_.begin() + prio + 1);
            prio--;
        }
    }

    //
    // Resize matrices and vectors
    //

    for(uint prio = 0; prio < constraint_vector_.size(); prio++)
    {
        uint no_constr_vars = 0;
        for(uint i = 0; i < constraint_vector_[prio].size(); i++)
        {
            ConstraintConfig conf = constraint_vector_[prio][i]->config;
            if(conf.type == jnt)
                no_constr_vars +=  conf.joint_names.size();
            else
                no_constr_vars += 6;
        }
        SolverInput solver_input_prio(no_constr_vars, no_robot_joints_);
        solver_input_.push_back(solver_input_prio);
    }

    configured_ = true;

    return true;
}


Constraint* WbcVelocity::constraint(const std::string &name)
{
    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    if(constraint_map_.count(name) == 0)
    {
        LOG_ERROR("No such constraint: %s", name.c_str());
        throw std::invalid_argument("Invalid constraint name");
    }
    return constraint_map_[name];
}


void WbcVelocity::prepareEqSystem(const std::vector<TaskFrame> &task_frames, std::vector<SolverInput> &solver_input){

    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    if(tf_map_.empty()){
        for(uint i = 0; i < task_frames.size(); i++)
            tf_map_[task_frames[i].tf_name] = TaskFrameKDL();
    }

    for(uint i = 0; i < task_frames.size(); i++)
    {
        if(task_frames[i].jac.cols() != no_robot_joints_){
            LOG_ERROR("Task frame with id %s has %i joints, but WbcVelocity has been configured to have %i joints ",
                      task_frames[i].tf_name.c_str(), task_frames[i].jac.cols(), no_robot_joints_);
            throw std::invalid_argument("Invalid task frame or WbcVelocityConfig");
        }

        TfToTfKDL(task_frames[i], tf_map_[task_frames[i].tf_name]);
    }

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < constraint_vector_.size(); prio++)
    {
        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraint_vector_[prio].size(); i++)
        {
            ExtendedConstraint *constraint = constraint_vector_[prio][i];          

            if(has_timeout_)
            {
                double diff = (base::Time::now() - constraint->last_ref_input).toSeconds();

                if( (diff > constraint_timeout_) &&
                   !(constraint->constraint_timed_out))
                    LOG_DEBUG("Constraint %s has timed out! No new reference has been received for %f seconds. Timeout is %f seconds",
                              constraint->config.name.c_str(), diff, constraint_timeout_);

                if(diff > constraint_timeout_)
                   constraint->constraint_timed_out = 1;
                else
                   constraint->constraint_timed_out = 0;
            }

            if(constraint->config.type == cart)
            {
                uint nc = 6; //constraint is Cartesian: always 6 task variables

                if(tf_map_.count(constraint->config.root) == 0){
                    LOG_ERROR("Root frame of constraint %s is %s, but this frame is not in task frame vector!", constraint->config.name.c_str(), constraint->config.root.c_str());
                    throw std::invalid_argument("Missing task frame");
                }
                if(tf_map_.count(constraint->config.tip) == 0){
                    LOG_ERROR("Task frame of constraint %s is %s, but this frame is not in task frame vector!", constraint->config.name.c_str(), constraint->config.tip.c_str());
                    throw std::invalid_argument("Missing task frame");
                }
                const TaskFrameKDL& tf_root = tf_map_[constraint->config.root];
                const TaskFrameKDL& tf_tip = tf_map_[constraint->config.tip];

                constraint->pose = tf_root.pose.Inverse() * tf_tip.pose;
                constraint->full_jac.data.setIdentity();
                constraint->full_jac.changeRefPoint(-constraint->pose.p);
                constraint->full_jac.changeRefFrame(tf_root.pose);

                //Invert constraint Jacobian
                KDL::svd_eigen_HH(constraint->full_jac.data, constraint->Uf, constraint->Sf, constraint->Vf, temp_);

                for (unsigned int j = 0; j < constraint->Sf.size(); j++)
                {
                    if (constraint->Sf(j) > 0)
                        constraint->Uf.col(j) *= 1 / constraint->Sf(j);
                    else
                        constraint->Uf.col(j).setZero();
                }
                constraint->H = (constraint->Vf * constraint->Uf.transpose());


                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root: Inverse constraint Jacobian * Robot Jacobian of object frame one
                constraint->A = constraint->H.block(0, 0, nc, 6) * tf_tip.jac.data
                        -(constraint->H.block(0, 0, nc, 6) * tf_root.jac.data);

                //If the constraint input is given in tip coordinates, convert to root
                if(constraint->config.ref_frame == constraint_ref_frame_tip)
                {
                    for(uint i = 0; i < 6; i++)
                        tw_(i) = constraint->y_des(i);
                    tw_ = constraint->pose.M * tw_;

                    for(uint i = 0; i < 6; i++)
                        constraint->y_des_root_frame(i) = tw_(i);
                }
                else
                    constraint->y_des_root_frame = constraint->y_des;
            }
            else if(constraint->config.type == jnt){
                for(uint i = 0; i < constraint->config.joint_names.size(); i++){

                    //Joint space constraints: constraint matrix has only ones and Zeros
                    //IMPORTANT: The joint order in the constraints might be different than in wbc.
                    //Thus, for joint space constraints, the joint indices have to be mapped correctly.
                    const std::string &joint_name = constraint->config.joint_names[i];
                    uint idx = joint_index_map_[joint_name];
                    constraint->A(i,idx) = 1.0;
                }
                constraint->y_des_root_frame = constraint->y_des;
            }

            constraint->time = base::Time::now();

            uint n_vars = constraint->no_variables;

            //insert constraint equation into equation system of current priority
            solver_input_[prio].Wy.segment(row_index, n_vars) = constraint->weights * constraint->activation * (!constraint->constraint_timed_out);
            solver_input_[prio].A.block(row_index, 0, n_vars, no_robot_joints_) = constraint->A;
            solver_input_[prio].y_ref.segment(row_index, n_vars) = constraint->y_des_root_frame;

            row_index += n_vars;
        }
    }

    solver_input = solver_input_;
}

std::vector<std::string> WbcVelocity::jointNames(){
    std::vector<std::string> joint_names(joint_index_map_.size());
    for(JointIndexMap::iterator it = joint_index_map_.begin(); it != joint_index_map_.end(); it++)
        joint_names[it->second] = it->first;
    return joint_names;
}
}
