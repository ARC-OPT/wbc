#include "WbcVelocity.hpp"
#include "SubTask.hpp"
#include "TaskFrame.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity(){
    configured_ = false;
    temp_ = Eigen::VectorXd(6);
}

WbcVelocity::~WbcVelocity(){
    clear();
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

}

bool WbcVelocity::configure(const KDL::Tree tree, const std::vector<SubTaskConfig> &config){

    clear();

    tree_ = tree;
    no_robot_joints_ = tree_.getNrOfJoints();

    LOG_DEBUG("No of joints is %i\n", no_robot_joints_);

    //
    // Create Subtasks and sort them by priority
    //
    uint max_prio = 0;
    for(uint i = 0; i < config.size(); i++){
        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    sub_task_vector_.resize(max_prio);
    for(uint i = 0; i < config.size(); i++){

        TaskFrame* tf_root = 0, *tf_tip = 0;
        if(config[i].type == task_type_cartesian){
            if(!addTaskFrame(config[i].root) ||
               !addTaskFrame((config[i].tip)))
                return false;

            tf_root = task_frame_map_[config[i].root];
            tf_tip = task_frame_map_[config[i].tip];
        }
        SubTask* sub_task = new SubTask(config[i], no_robot_joints_, tf_root, tf_tip);
        sub_task_vector_[config[i].priority].push_back(sub_task);

        //Also put subtasks in a map that associates them with their names (for easier access)
        if(sub_task_map_.count(config[i].name) != 0){
            LOG_ERROR("Task with name %s already exists! Task names must be unique", config[i].name.c_str());
            return false;
        }
        sub_task_map_[config[i].name] = sub_task;
    }

    //Erase empty priorities
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++){
        if(sub_task_vector_[prio].empty()){
            sub_task_vector_.erase(sub_task_vector_.begin() + prio, sub_task_vector_.begin() + prio + 1);
            prio--;
        }
    }

    LOG_DEBUG("Sub Task Vector: ");
    for(uint i = 0; i < sub_task_vector_.size(); i++){
        LOG_DEBUG("Prio: %i", i);
        for(uint j = 0; j < sub_task_vector_[i].size(); j++){
            LOG_DEBUG("Task name: %s", sub_task_vector_[i][j]->config_.name.c_str());
            LOG_DEBUG("No task vars: %i", sub_task_vector_[i][j]->y_des_.size());
        }
    }
    LOG_DEBUG("Sub Task Map: ");
    for(SubTaskMap::iterator it = sub_task_map_.begin(); it != sub_task_map_.end(); it++)
        LOG_DEBUG("Prio: %s", it->first.c_str());

    //
    // Resize matrices and vectors
    //
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++){
        uint no_task_vars_prio = 0;
        for(uint i = 0; i < sub_task_vector_[prio].size(); i++){

            SubTaskConfig conf = sub_task_vector_[prio][i]->config_;
            uint type = conf.type;
            if(type == task_type_cartesian)
                no_task_vars_prio += 6;
            else
                no_task_vars_prio +=  conf.joints.size();
        }

        Eigen::MatrixXd A(no_task_vars_prio, no_robot_joints_);
        Eigen::VectorXd y(no_task_vars_prio);
        Eigen::VectorXd y_act(no_task_vars_prio);
        Eigen::VectorXd Wy(no_task_vars_prio);

        Wy.setConstant(1); //Set all task weights to 1 in the beginning
        A.setZero();
        y_act.setZero();
        A_.push_back(A);
        y_ref_.push_back(y);
        Wy_.push_back(Wy);
    }

    configured_ = true;
    return true;
}

bool WbcVelocity::addTaskFrame(const std::string &frame_id){
    if(task_frame_map_.count(frame_id) == 0){
        std::string robot_root = tree_.getRootSegment()->first;
        KDL::Chain chain;
        if(!tree_.getChain(robot_root, frame_id, chain)){
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root.c_str(), frame_id.c_str());
            return false;
        }
        TaskFrame* tf = new TaskFrame(chain, no_robot_joints_);
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

SubTask* WbcVelocity::subTask(const std::string &name){

    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    if(sub_task_map_.count(name) == 0){
        std::stringstream ss;
        ss<<"No such sub Task: "<<name<<endl;
        throw std::invalid_argument(ss.str());
    }
    return sub_task_map_[name];
}


void WbcVelocity::update(const base::samples::Joints &status){

    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    //if not done yet create joint index vector
    if(joint_index_map_.empty()){
        for(uint i = 0; i < status.size(); i++)
            joint_index_map_[status.names[i]] = i;
    }

    //Update Task Frames
    for(TaskFrameMap::iterator it = task_frame_map_.begin(); it != task_frame_map_.end(); it++)
         it->second->update(status);

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++){


        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < sub_task_vector_[prio].size(); i++){

            SubTask *sub_task = sub_task_vector_[prio][i];
            if(sub_task->config_.type == task_type_cartesian){
                uint nc = 6; //Task is Cartesian: always 6 task variables
                sub_task->pose_ = sub_task->tf_root_->pose_.Inverse() * sub_task->tf_tip_->pose_;
                sub_task->task_jac_.data.setIdentity();
                sub_task->task_jac_.changeRefPoint(-sub_task->pose_.p);
                sub_task->task_jac_.changeRefFrame(sub_task->tf_root_->pose_);

                //Invert Task Jacobian
                KDL::svd_eigen_HH(sub_task->task_jac_.data, sub_task->Uf_, sub_task->Sf_, sub_task->Vf_, temp_);

                for (unsigned int j = 0; j < sub_task->Sf_.size(); j++){
                    if (sub_task->Sf_(j) > 0)
                        sub_task->Uf_.col(j) *= 1 / sub_task->Sf_(j);
                    else
                        sub_task->Uf_.col(j).setZero();
                }
                sub_task->H_ = (sub_task->Vf_ * sub_task->Uf_.transpose());


                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root: Inverse Task Jacobian * Robot Jacobian of object frame one
                sub_task->A_ = sub_task->H_.block(0, 0, nc, 6) * sub_task->tf_tip_->jac_robot_.data
                        -(sub_task->H_.block(0, 0, nc, 6) * sub_task->tf_root_->jac_robot_.data);

            }
            else if(sub_task->config_.type == task_type_joint){
                for(uint i = 0; i < sub_task->config_.joints.size(); i++){
                    std::string joint_name = sub_task->config_.joints[i];
                    uint idx = joint_index_map_[joint_name];
                    sub_task->A_(i,idx) = 1.0;
                }
            }
            uint n_vars = sub_task->no_task_vars_;

            //insert task equation into equation system of current priority
            Wy_[prio].segment(row_index, n_vars) = sub_task->task_weights_;
            y_ref_[prio].segment(row_index, n_vars) = sub_task->y_des_;
            A_[prio].block(row_index, 0, n_vars, no_robot_joints_) = sub_task->A_;

            row_index += n_vars;
        }
    }
}

}
