#include "WbcVelocity.hpp"
#include "ExtendedConstraint.hpp"
#include "TaskFrame.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>
#include <wbc/ExtendedConstraint.hpp>
#include "HierarchicalWDLSSolver.hpp"

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity() :
    configured_(false),
    temp_(Eigen::VectorXd(6)),
    no_robot_joints_(0){
    solver_ = new HierarchicalWDLSSolver();
}

WbcVelocity::~WbcVelocity(){
    clear();
    delete solver_;
}

void WbcVelocity::clear(){

    for(uint prio = 0; prio < constraint_vector_.size(); prio++ ){
        for(uint j = 0; j < constraint_vector_[prio].size(); j++)
            delete constraint_vector_[prio][j];
        constraint_vector_[prio].clear();
    }

    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
        delete it->second;

    constraint_map_.clear();
    constraint_vector_.clear();
    task_frame_map_.clear();
    joint_index_map_.clear();
    A_.clear();
    y_ref_.clear();
    Wy_.clear();
    no_robot_joints_ = 0;
}

bool WbcVelocity::configure(const KDL::Tree tree,
                            const std::vector<ConstraintConfig> &config,
                            const std::vector<std::string> &joint_names,
                            bool constraints_active,
                            double constraints_timeout,
                            bool debug){

    clear();

    debug_ = debug;
    constraint_timeout_ = constraints_timeout;
    has_timeout_ = true;
    if(base::isUnset(constraints_timeout))
        has_timeout_ = false;

    robot_root_ = tree.getRootSegment()->first;
    tree_ = tree;
    no_robot_joints_ = tree_.getNrOfJoints();

    if(no_robot_joints_ == 0)
    {
        LOG_ERROR("KDL tree contains zero joints");
        return false;
    }

    //
    // Create joint index map
    //

    // The joint name property can define the internal order of joints.
    // This order will be kept in all constraint matrices, weight matrices, status vectors, etc.
    // If no joint names are given, the order will be the same as in the KDL tree
    joint_index_map_.clear();
    if(joint_names.empty())
    {
        KDL::SegmentMap segments = tree_.getSegments();
        uint idx = 0;
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
                joint_index_map_[seg.getJoint().getName()] = idx++;
        }
    }
    else
    {
        for(uint i = 0; i < joint_names.size(); i++)
            joint_index_map_[joint_names[i]] = i;

        //Check if all joints in tree are in joint index map
        KDL::SegmentMap segments = tree_.getSegments();
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
            {
                if(joint_index_map_.count(seg.getJoint().getName()) == 0)
                {
                    LOG_ERROR("Joint with name %s is in KDL::Tree but not in joint names parameter", seg.getJoint().getName().c_str());
                    LOG_ERROR("If the order of joints shall be fixed with the joint names parameter, all joints in tree have to be given here");
                    return false;
                }
            }
        }
    }

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
        ExtendedConstraint* constraint = 0;

        switch(config[i].type){
        case cart:{
            if(config[i].root.empty() || config[i].tip.empty())
            {
                LOG_ERROR("Constraint %s has empty root or tip frame name", config[i].name.c_str());
                return false;
            }
            if(!addTaskFrame(config[i].root) ||
               !addTaskFrame((config[i].tip)))
                return false;

            KDL::Chain chain;
            tree_.getChain(config[i].root, config[i].tip, chain);
            constraint = new ExtendedConstraint(config[i], chain, no_robot_joints_, constraints_active);

            break;
        }
        case jnt:{
            if(config[i].joint_names.size() == 0)
            {
                LOG_ERROR("Constraint %s has joint names given. Check your configuration!", config[i].name.c_str());
                return false;
            }
            constraint = new ExtendedConstraint(config[i], no_robot_joints_, constraints_active);
            break;
        }
        default:{
            LOG_ERROR("Unknown constraint type: %i", config[i].type);
            throw std::invalid_argument("Invalid constraint type");
            break;
        }
        }

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

    std::vector<uint> no_constr_vars_pp;
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

        Eigen::MatrixXd A(no_constr_vars, no_robot_joints_);
        Eigen::VectorXd y_ref(no_constr_vars);
        Eigen::VectorXd Wy(no_constr_vars);

        Wy.setIdentity(); //Set all task weights to 1 in the beginning
        A.setZero();
        y_ref.setZero();
        A_.push_back(A);
        y_ref_.push_back(y_ref);
        Wy_.push_back(Wy);
        no_constr_vars_pp.push_back(y_ref.size());
    }

    configured_ = true;

    if(!solver_->configure(no_constr_vars_pp, no_robot_joints_))
        return false;

    return true;
}

bool WbcVelocity::addTaskFrame(const std::string &frame_id){

    if(task_frame_map_.count(frame_id) == 0)
    {
        KDL::Chain chain;
        if(!tree_.getChain(robot_root_, frame_id, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root_.c_str(), frame_id.c_str());
            return false;
        }
        TaskFrame* tf = new TaskFrame(chain, no_robot_joints_, joint_index_map_);
        task_frame_map_[frame_id] = tf;

        LOG_DEBUG("Sucessfully added task frame %s", frame_id.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
    }
    else
        LOG_INFO("Task Frame with id %s has already been added", frame_id.c_str());

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


void WbcVelocity::solve(const base::samples::Joints &status, Eigen::VectorXd &ctrl_out){

    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    //Update Task Frames
    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
         it->second->update(status);

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

            if(constraint->config.type == cart){
                uint nc = 6; //constraint is Cartesian: always 6 task variables
                TaskFrame* tf_root = task_frame_map_[constraint->config.root];
                TaskFrame* tf_tip = task_frame_map_[constraint->config.tip];
                constraint->pose = tf_root->pose_.Inverse() * tf_tip->pose_;
                constraint->full_jac.data.setIdentity();
                constraint->full_jac.changeRefPoint(-constraint->pose.p);
                constraint->full_jac.changeRefFrame(tf_root->pose_);

                //Compute manipulability index in debug mode
                if(debug_)
                    constraint->manipulability = constraint->computeManipulability(status);

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
                constraint->A = constraint->H.block(0, 0, nc, 6) * tf_tip->jac_robot_.data
                        -(constraint->H.block(0, 0, nc, 6) * tf_root->jac_robot_.data);

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
                if(debug_)
                    constraint->manipulability = base::NaN<double>();
            }

            constraint->time = base::Time::now();

            uint n_vars = constraint->no_variables;

            //insert constraint equation into equation system of current priority
            Wy_[prio].segment(row_index, n_vars) = constraint->weights * constraint->activation * (!constraint->constraint_timed_out);
            A_[prio].block(row_index, 0, n_vars, no_robot_joints_) = constraint->A;
            y_ref_[prio].segment(row_index, n_vars) = constraint->y_des_root_frame;

            row_index += n_vars;
        }

        solver_->setTaskWeights(Wy_[prio], prio);
    }

    solver_->solve(A_, y_ref_, ctrl_out);
}

std::vector<std::string> WbcVelocity::jointNames(){
    std::vector<std::string> joint_names(joint_index_map_.size());
    for(JointIndexMap::iterator it = joint_index_map_.begin(); it != joint_index_map_.end(); it++)
        joint_names[it->second] = it->first;
    return joint_names;
}
}
