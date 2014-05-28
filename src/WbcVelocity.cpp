#include "WbcVelocity.hpp"
#include "ExtendedSubTask.hpp"
#include "TaskFrame.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>
#include <wbc/ExtendedSubTask.hpp>
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

    for(uint prio = 0; prio < sub_task_vector_.size(); prio++ ){
        for(uint j = 0; j < sub_task_vector_[prio].size(); j++)
            delete sub_task_vector_[prio][j];
        sub_task_vector_[prio].clear();
    }

    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
        delete it->second;

    sub_task_map_.clear();
    sub_task_vector_.clear();
    task_frame_map_.clear();
    joint_index_map_.clear();
    A_.clear();
    y_ref_.clear();
    Wy_.clear();
    no_robot_joints_ = 0;
}

bool WbcVelocity::configure(const KDL::Tree tree,
                            const std::vector<SubTaskConfig> &config,
                            const std::vector<std::string> &joint_names,
                            bool tasks_active,
                            double task_timeout,
                            bool debug){

    clear();

    debug_ = debug;
    task_timeout_ = task_timeout;
    has_timeout_ = true;
    if(base::isUnset(task_timeout))
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
    // This order will be kept in all task matrices, weight matrices, status vectors, etc.
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
    // Create Subtasks and sort them by priority
    //
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++)
    {
        if(config[i].priority < 0)
        {
            LOG_ERROR("Task Priorities must be >= 0. Task priority of task %s is %i", config[i].name.c_str(), config[i].priority);
            return false;
        }
        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    sub_task_vector_.resize(max_prio + 1);
    for(uint i = 0; i < config.size(); i++)
    {
        ExtendedSubTask* sub_task = 0;

        switch(config[i].type){
        case cart:{
            if(config[i].root.empty() || config[i].tip.empty())
            {
                LOG_ERROR("Task %s has empty root or tip frame name", config[i].name.c_str());
                return false;
            }
            if(!addTaskFrame(config[i].root) ||
               !addTaskFrame((config[i].tip)))
                return false;

            if(config[i].task_var_names.size() != 6)
            {
                LOG_ERROR("Task %s is Cartesian but size of task variables is not 6. Check your configuration!", config[i].name.c_str());
                return false;
            }

            KDL::Chain chain;
            tree_.getChain(config[i].root, config[i].tip, chain);
            sub_task = new ExtendedSubTask(config[i], chain, no_robot_joints_, tasks_active);

            break;
        }
        case jnt:{
            if(config[i].task_var_names.size() == 0)
            {
                LOG_ERROR("Task %s has no task variable name given. Check your configuration!", config[i].name.c_str());
                return false;
            }
            sub_task = new ExtendedSubTask(config[i], no_robot_joints_, tasks_active);
            break;
        }
        default:{
            LOG_ERROR("Unknown task type: %i", config[i].type);
            throw std::invalid_argument("Invalid task type");
            break;
        }
        }

        sub_task_vector_[config[i].priority].push_back(sub_task);

        //Also put subtasks in a map that associates them with their names (for easier access)
        if(sub_task_map_.count(config[i].name) != 0)
        {
            LOG_ERROR("Task with name %s already exists! Task names must be unique", config[i].name.c_str());
            return false;
        }
        sub_task_map_[config[i].name] = sub_task;
    }

    //Erase empty priorities
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++)
    {
        if(sub_task_vector_[prio].empty())
        {
            sub_task_vector_.erase(sub_task_vector_.begin() + prio, sub_task_vector_.begin() + prio + 1);
            prio--;
        }
    }

    //
    // Resize matrices and vectors
    //

    std::vector<uint> no_task_vars_pp;
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++)
    {
        uint no_task_vars_prio = 0;
        for(uint i = 0; i < sub_task_vector_[prio].size(); i++)
        {
            SubTaskConfig conf = sub_task_vector_[prio][i]->config;
            no_task_vars_prio +=  conf.task_var_names.size();
        }

        Eigen::MatrixXd A(no_task_vars_prio, no_robot_joints_);
        Eigen::VectorXd y_ref(no_task_vars_prio);
        Eigen::MatrixXd Wy(no_task_vars_prio, no_task_vars_prio);

        Wy.setIdentity(); //Set all task weights to 1 in the beginning
        A.setZero();
        y_ref.setZero();
        A_.push_back(A);
        y_ref_.push_back(y_ref);
        Wy_.push_back(Wy);
        no_task_vars_pp.push_back(y_ref.size());
    }

    configured_ = true;

    if(!solver_->configure(no_task_vars_pp, no_robot_joints_))
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

SubTask* WbcVelocity::subTask(const std::string &name)
{
    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    if(sub_task_map_.count(name) == 0)
    {
        LOG_ERROR("No such sub task: %s", name.c_str());
        throw std::invalid_argument("Invalid task name");
    }
    return sub_task_map_[name];
}


void WbcVelocity::solve(const base::samples::Joints &status, Eigen::VectorXd &ctrl_out){

    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    //Update Task Frames
    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
         it->second->update(status);

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++)
    {
        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < sub_task_vector_[prio].size(); i++)
        {
            ExtendedSubTask *sub_task = sub_task_vector_[prio][i];

            if(has_timeout_)
            {
                double diff = (base::Time::now() - sub_task->last_task_input).toSeconds();

                if( (diff > task_timeout_) &&
                   !(sub_task->task_timed_out))
                    LOG_DEBUG("Task %s has timed out! No new reference has been received for %f seconds. Timeout is %f seconds", sub_task->config.name.c_str(), diff, task_timeout_);

                if(diff > task_timeout_)
                   sub_task->task_timed_out = 1;
                else
                   sub_task->task_timed_out = 0;
            }

            if(sub_task->config.type == cart){
                uint nc = 6; //Task is Cartesian: always 6 task variables
                TaskFrame* tf_root = task_frame_map_[sub_task->config.root];
                TaskFrame* tf_tip = task_frame_map_[sub_task->config.tip];
                sub_task->pose = tf_root->pose_.Inverse() * tf_tip->pose_;
                sub_task->task_jac.data.setIdentity();
                sub_task->task_jac.changeRefPoint(-sub_task->pose.p);
                sub_task->task_jac.changeRefFrame(tf_root->pose_);

                //Compute manipulability index in debug mode
                if(debug_)
                    sub_task->manipulability = sub_task->computeManipulability(status);

                //Invert Task Jacobian
                KDL::svd_eigen_HH(sub_task->task_jac.data, sub_task->Uf, sub_task->Sf, sub_task->Vf, temp_);

                for (unsigned int j = 0; j < sub_task->Sf.size(); j++)
                {
                    if (sub_task->Sf(j) > 0)
                        sub_task->Uf.col(j) *= 1 / sub_task->Sf(j);
                    else
                        sub_task->Uf.col(j).setZero();
                }
                sub_task->H = (sub_task->Vf * sub_task->Uf.transpose());


                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root: Inverse Task Jacobian * Robot Jacobian of object frame one
                sub_task->A = sub_task->H.block(0, 0, nc, 6) * tf_tip->jac_robot_.data
                        -(sub_task->H.block(0, 0, nc, 6) * tf_root->jac_robot_.data);

                //If the task input is given in tip coordinates, convert to root
                if(sub_task->config.ref_frame == task_ref_frame_tip)
                {
                    for(uint i = 0; i < 6; i++)
                        tw_(i) = sub_task->y_des(i);
                    tw_ = sub_task->pose.M * tw_;

                    for(uint i = 0; i < 6; i++)
                        sub_task->y_des_root_frame(i) = tw_(i);
                }
                else
                    sub_task->y_des_root_frame = sub_task->y_des;
            }
            else if(sub_task->config.type == jnt){
                for(uint i = 0; i < sub_task->config.task_var_names.size(); i++){

                    //Joint space tasks: Task matrix has only ones and Zeros
                    //IMPORTANT: The joint order in the tasks might be different than in wbc.
                    //Thus, for joint space tasks, the joint indices have to be mapped correctly.
                    const std::string &joint_name = sub_task->config.task_var_names[i];
                    uint idx = joint_index_map_[joint_name];
                    sub_task->A(i,idx) = 1.0;
                }
                sub_task->y_des_root_frame = sub_task->y_des;
                if(debug_)
                    sub_task->manipulability = base::NaN<double>();
            }

            sub_task->time = base::Time::now();

            uint n_vars = sub_task->no_task_vars;

            //insert task equation into equation system of current priority
            Wy_[prio].block(row_index, row_index, n_vars, n_vars).diagonal() = sub_task->weights * sub_task->activation * (!sub_task->task_timed_out);
            A_[prio].block(row_index, 0, n_vars, no_robot_joints_) = sub_task->A;
            y_ref_[prio].segment(row_index, n_vars) = sub_task->y_des_root_frame;

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
