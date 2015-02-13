#include "WbcVelocity.hpp"
#include "ExtendedConstraint.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity() :
    configured_(false),
    no_robot_joints_(0),
    temp_(Eigen::VectorXd(6)){
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
    jac_map_.clear();
    constraint_map_.clear();
    constraint_vector_.clear();
    joint_index_map_.clear();
    no_robot_joints_ = 0;
}

bool WbcVelocity::configure(const std::vector<ConstraintConfig> &config,
                            const std::vector<std::string> &joint_names,
                            double constraint_timeout){

    //Erase constraints, jacobians and joint indices
    clear();

    constraint_timeout_ = constraint_timeout;
    has_timeout_ = true;
    if(base::isUnset(constraint_timeout))
        has_timeout_ = false;
    else
    {
        if(constraint_timeout <= 0){
            LOG_ERROR("Constrainted timeout should be a value > 0, but is %f", constraint_timeout);
            return false;
        }
    }

    no_robot_joints_ = joint_names.size();

    // Create joint index map. This defines the order of the joints in the task Jacobians computed here. Note
    // that the order of joints in the task frames can be different.
    for(uint i = 0; i < joint_names.size(); i++)
        joint_index_map_[joint_names[i]] = i;

    // Create Constraints and sort them by priority
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++)
    {
        if(config[i].priority < 0)
        {
            LOG_ERROR("Constraint Priorities must be >= 0. Constraint priority of constraint '%s'' is %i", config[i].name.c_str(), config[i].priority);
            return false;
        }
        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    constraint_vector_.resize(max_prio + 1);
    for(uint i = 0; i < config.size(); i++)
    {
        ExtendedConstraint* constraint = new ExtendedConstraint(config[i], joint_names);
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

    n_constraints_per_prio_ = std::vector<int>(constraint_vector_.size(), 0);
    n_prios_ = n_constraints_per_prio_.size();
    for(uint prio = 0; prio < constraint_vector_.size(); prio++)
    {
        for(uint j = 0; j < constraint_vector_[prio].size(); j++)
            n_constraints_per_prio_[prio] += constraint_vector_[prio][j]->y_ref.size();
    }

    //Create TF map
    for(ConstraintMap::iterator it = constraint_map_.begin(); it != constraint_map_.end(); it++)
    {
        if(it->second->config.type == cart){

            std::string tf_name = it->second->config.root;
            if(tf_map_.count(tf_name) == 0)
            {
                tf_map_[tf_name] = TaskFrameKDL();
                KDL::Jacobian jac(joint_index_map_.size());
                jac.data.setZero();
                jac_map_[tf_name] = jac;
            }

            tf_name = it->second->config.tip;
            if(tf_map_.count(tf_name) == 0)
            {
                tf_map_[tf_name] = TaskFrameKDL();
                KDL::Jacobian jac(joint_index_map_.size());
                jac.data.setZero();
                jac_map_[tf_name] = jac;
            }

            tf_name = it->second->config.ref_frame;
            if(tf_map_.count(tf_name) == 0)
            {
                tf_map_[tf_name] = TaskFrameKDL();
                KDL::Jacobian jac(joint_index_map_.size());
                jac.data.setZero();
                jac_map_[tf_name] = jac;
            }
        }
    }

    LOG_DEBUG("Joint Index Map: \n");
    for(JointIndexMap::iterator it = joint_index_map_.begin(); it != joint_index_map_.end(); it++)
        LOG_DEBUG_S<<it->first.c_str()<<": "<<it->second;

    LOG_DEBUG("\nTask Frame Map: \n");
    for(TaskFrameKDLMap::iterator it = tf_map_.begin(); it != tf_map_.end(); it++)
        LOG_DEBUG_S<<it->first.c_str();

    LOG_DEBUG("\nConstraint Map: \n");
    for(ConstraintMap::iterator it = constraint_map_.begin(); it != constraint_map_.end(); it++)
        LOG_DEBUG_S<<it->first.c_str()<<": prio: "<<it->second->config.priority<<", type: "<<it->second->config.type;

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

void WbcVelocity::prepareEqSystem(const std::vector<TaskFrameKDL> &task_frames, std::vector<LinearEqnSystem> &equations)
{
    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    //Check if all required task frames are available
    for(TaskFrameKDLMap::iterator it = tf_map_.begin(); it != tf_map_.end(); it++)
    {
        bool found = false;
        for(uint i = 0; i < task_frames.size(); i++)
        {
            if(it->first.compare(task_frames[i].tf_name_) == 0){
                found = true;
                tf_map_[task_frames[i].tf_name_] = task_frames[i];
            }
        }

        if(!found){
            LOG_ERROR("Wbc config requires Task Frame %s, but this task frame is not in task_frame vector", it->first.c_str());
            throw std::invalid_argument("Incomplete task frame input");
        }
    }

    //update task frame map and create full robot Jacobians
    for(uint i = 0; i < task_frames.size(); i++)
    {
        const TaskFrameKDL &tf = task_frames[i];
        tf_map_[tf.tf_name_] = tf;

        //IMPORTANT: Fill in columns of task frame Jacobian into the correct place of the full robot Jacobian using the joint_index_map
        for(uint j = 0; j < tf.joint_names_.size(); j++)
        {
            const std::string& tf_name = tf.tf_name_;
            const std::string& jt_name =  tf.joint_names_[j];

            if(joint_index_map_.count(jt_name) == 0)
            {
                LOG_ERROR("Joint with id %s does not exist in joint index map. Check your joint_names configuration!", jt_name.c_str());
                throw std::invalid_argument("Invalid joint name");
            }

            uint idx = joint_index_map_[jt_name];
            jac_map_[tf_name].setColumn(idx, tf_map_[tf_name].jac_.getColumn(j));
        }
    }

    if(equations.size() != n_prios_)
        equations.resize(n_prios_);

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < n_prios_; prio++)
    {
        uint n_vars_prio = n_constraints_per_prio_[prio]; //Number of constraint variables on the whole priority

        if(equations[prio].A.rows() != n_vars_prio ||
                equations[prio].A.cols() != no_robot_joints_ ||
                equations[prio].y_ref.size() != n_vars_prio ||
                equations[prio].W_row.size() != n_vars_prio ||
                equations[prio].W_col.size() != no_robot_joints_)
        {
            equations[prio].resize(n_vars_prio, no_robot_joints_);
        }

        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraint_vector_[prio].size(); i++)
        {
            ExtendedConstraint *constraint = constraint_vector_[prio][i];
            const uint n_vars = constraint->no_variables;

            //Check task timeout
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
                const TaskFrameKDL& tf_root = tf_map_[constraint->config.root];
                const TaskFrameKDL& tf_tip = tf_map_[constraint->config.tip];
                const TaskFrameKDL& tf_ref_frame = tf_map_[constraint->config.ref_frame];

                //Create constraint jacobian
                constraint->pose = tf_root.pose_.Inverse() * tf_tip.pose_;
                constraint->full_jac.data.setIdentity();
                constraint->full_jac.changeRefPoint(-constraint->pose.p);
                constraint->full_jac.changeRefFrame(tf_root.pose_);

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

                const KDL::Jacobian& jac_root =  jac_map_[tf_root.tf_name_];
                const KDL::Jacobian& jac_tip =  jac_map_[tf_tip.tf_name_];

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
                for(uint i = 0; i < 6; i++)
                    tw_(i) = constraint->y_ref(i);
                tw_ = (tf_root.pose_.M.Inverse() * tf_ref_frame.pose_.M) * tw_;
                for(uint i = 0; i < 6; i++)
                    constraint->y_ref_root(i) = tw_(i);
            }
            else if(constraint->config.type == jnt){
                for(uint i = 0; i < constraint->config.joint_names.size(); i++){

                    //Joint space constraints: constraint matrix has only ones and Zeros
                    //IMPORTANT: The joint order in the constraints might be different than in wbc.
                    //Thus, for joint space constraints, the joint indices have to be mapped correctly.
                    const std::string &joint_name = constraint->config.joint_names[i];
                    if(joint_index_map_.count(joint_name) == 0)
                    {
                        LOG_ERROR("Constraint %s contains joint %s, but this joint has not been configured in joint names", constraint->config.name.c_str(), joint_name.c_str());
                        throw std::invalid_argument("Invalid Constraint config");
                    }
                    uint idx = joint_index_map_[joint_name];
                    constraint->A(i,idx) = 1.0;
                    constraint->y_ref_root = constraint->y_ref; // In joint space, y_ref is of yourse equal to y_ref_root
                }
            }

            constraint->time = base::Time::now();

            // Insert constraints into equation system of current priority at the correct position
            equations[prio].W_row.segment(row_index, n_vars) = constraint->weights * constraint->activation * (!constraint->constraint_timed_out);
            equations[prio].A.block(row_index, 0, n_vars, no_robot_joints_) = constraint->A;
            equations[prio].y_ref.segment(row_index, n_vars) = constraint->y_ref_root;

            row_index += n_vars;
        }
    }
}

std::vector<std::string> WbcVelocity::jointNames(){
    std::vector<std::string> joint_names(joint_index_map_.size());
    for(JointIndexMap::iterator it = joint_index_map_.begin(); it != joint_index_map_.end(); it++)
        joint_names[it->second] = it->first;
    return joint_names;
}

std::vector<std::string> WbcVelocity::getTaskFrameIDs()
{
    std::vector<std::string> task_frame_ids;
    TaskFrameKDLMap::iterator it;
    for(it = tf_map_.begin(); it != tf_map_.end(); it++)
        task_frame_ids.push_back(it->first);
    return task_frame_ids;
}

void WbcVelocity::getConstraintVector(std::vector<ConstraintsPerPrio>& constraints)
{
    if(constraints.size() != n_prios_)
        constraints.resize(n_prios_);
    for(uint prio = 0; prio < n_prios_; prio++)
    {
        if(constraints[prio].size() != constraint_vector_[prio].size())
            constraints[prio].resize(constraint_vector_[prio].size());

        for(uint i = 0; i < constraints[prio].size(); i++)
            constraints[prio][i] = *constraint_vector_[prio][i];
    }
}

}
