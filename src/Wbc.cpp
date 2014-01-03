#include "Wbc.hpp"
#include "TaskFrame.hpp"
#include "SubTask.hpp"
#include "RobotModel.hpp"
#include "HierarchicalWDLSSolver.hpp"

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>

using namespace std;

Wbc::Wbc(KDL::Tree tree, const WBC_TYPE wbc_type){
    configured_ = false;
    robot_ = new RobotModel(tree);
    wbc_type_= wbc_type;
    temp_ = Eigen::VectorXd(6);

    switch(wbc_type_){
    case WBC_TYPE_VELOCITY:{
        solver_ = new HierarchicalWDLSSolver();
        break;
    }
    case WBC_TYPE_TORQUE:{
        break;
    }
    default:{
        LOG_ERROR("Invalid WBC type: %i", wbc_type_);
        throw std::invalid_argument("Invalid wbc type");
        break;
    }
    }
}

Wbc::~Wbc(){
    for(uint i = 0; i < sub_task_vector_.size(); i++ )
        delete sub_task_vector_[i];

    sub_task_vector_.clear();
    sub_task_map_.clear();

    delete solver_;
    delete robot_;
}

bool Wbc::addSubTask(const std::string &task_name,
                     const uint priority,
                     const std::string& root,
                     const std::string& tip,
                     const uint no_task_variables){

    if(sub_task_map_.count(task_name) > 0){
        LOG_ERROR("Sub task with name %s already exist", task_name.c_str());
        return false;
    }

    //Configure will have to be called if a new sub task is added
    configured_ = false;

    //Add root frame to robot model
    if(!root.empty()){
        if(!robot_->addTaskFrame(root))
            return false;
    }

    //Add tip frame to robot model
    if(!tip.empty()){
        if(!robot_->addTaskFrame(tip))
            return false;
    }

    //Create and insert sub task into sub task map at the correct position (priority ordered)
    SubTask* sub_task = new SubTask(root, tip, no_task_variables, robot_->no_of_joints_, priority);
    uint i = 0;
    for(i = 0; i < sub_task_vector_.size(); i++){
        if(sub_task_vector_[i]->priority_ > priority)
            break;
    }
    sub_task_vector_.insert(sub_task_vector_.begin() + i, sub_task);
    sub_task_map_[task_name] = sub_task;

    LOG_DEBUG("Successfully added Sub Task %s", task_name.c_str());
    LOG_DEBUG("Sub task Map now contains:");
    for(SubTaskMap::iterator it = sub_task_map_.begin(); it != sub_task_map_.end(); it++)
        LOG_DEBUG("%s", it->first.c_str());
    LOG_DEBUG("\n");

    //if this priority is already available add no of task variables to this priority
    if(priority_map_.count(priority) > 0)
        priority_map_[priority] += no_task_variables;
    else
        priority_map_[priority] = no_task_variables;


    LOG_DEBUG("Updated priority map");
    LOG_DEBUG("Priority Map now contains:");
    for(PriorityMap::iterator it = priority_map_.begin(); it != priority_map_.end(); it++)
        LOG_DEBUG("Priority: %i Task variables: %i", it->first, it->second);
    LOG_DEBUG("\n");

    return true;
}

bool Wbc::configure(){

    if(sub_task_vector_.empty())
        throw std::runtime_error("No sub tasks have been added to wbc");

    //Create A and y matrices that describe the equation systems for the solver
    std::vector<uint> ny_per_priority;
    for(PriorityMap::iterator it = priority_map_.begin(); it != priority_map_.end(); it++)
        ny_per_priority.push_back(it->second);

    for(uint i = 0; i < ny_per_priority.size(); i++){
        Eigen::MatrixXd A(ny_per_priority[i], robot_->no_of_joints_);
        Eigen::VectorXd y(ny_per_priority[i]);
        A.setZero();
        y.setZero();
        A_.push_back(A);
        y_.push_back(y);
        cout<<A<<endl;
        cout<<y<<endl;
    }

    if(!solver_->configure(ny_per_priority, robot_->no_of_joints_))
        return false;

    LOG_DEBUG("Configured solver:");
    for(uint i = 0; i < ny_per_priority.size(); i++)
        LOG_DEBUG("Priority: %i Task variables: %i", i, ny_per_priority[i]);
    LOG_DEBUG("No of robot joints: %i\n", robot_->no_of_joints_);

    solver_output_.resize(robot_->no_of_joints_);

    configured_ = true;
    return true;
}


void Wbc::updateSubTask(const std::string &task_name, const Eigen::VectorXd &y_des){
    if(sub_task_map_.count(task_name) == 0){
        LOG_ERROR("No such sub task: %s", task_name.c_str());
        throw std::invalid_argument("Invalid sub task name");
    }

    SubTask *sub_task = sub_task_map_[task_name];
    if(sub_task->no_task_variables_ != y_des.cols()){
        LOG_ERROR("No of task variables of task %s is %i, but size of desired task state is %i", task_name.c_str(), sub_task->no_task_variables_, y_des.cols());
        throw std::invalid_argument("Invalid no of task variables in wbc input");
    }
    sub_task->y_des_ = y_des;

    switch(wbc_type_){
    case WBC_TYPE_VELOCITY:{

        //Compute Pose of sub task tip frame wrt root frame
        if(!sub_task->root_.empty() && !sub_task->tip_.empty()){

            TaskFrame* tf_root = robot_->getTaskFrame(sub_task->root_);
            TaskFrame* tf_tip = robot_->getTaskFrame(sub_task->tip_);

            if(tf_root && tf_tip){ //Task is in Cartesian space
                sub_task->pose_ = tf_root->pose_.Inverse() * tf_tip->pose_;
                sub_task->task_jac_.data.setIdentity();
                sub_task->task_jac_.changeRefPoint(-sub_task->pose_.p);
                sub_task->task_jac_.changeRefFrame(tf_root->pose_);

                //Invert Task Jacobian
                if (KDL::svd_eigen_HH(sub_task->task_jac_.data, sub_task->Uf_, sub_task->Sf_, sub_task->Vf_, temp_) != 0) {
                    LOG_ERROR(" Could not invert task jacobian of task %s", task_name.c_str());
                    throw std::runtime_error("Could not invert task jacobian");
                }

                for (unsigned int j = 0; j < sub_task->Sf_.size(); j++){
                    if (sub_task->Sf_(j) > 0)
                        sub_task->Uf_.col(j) *= 1 / sub_task->Sf_(j);
                    else
                        sub_task->Uf_.col(j).setZero();
                }
                sub_task->H_ = (sub_task->Vf_ * sub_task->Uf_.transpose());

                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root: Inverse Task Jacobian * Robot Jacobian of object frame one
                sub_task->A_ = sub_task->H_.block(0, 0, sub_task->no_task_variables_, 6) * tf_tip->jac_robot_.data
                        -(sub_task->H_.block(0, 0, sub_task->no_task_variables_, 6) * tf_root->jac_robot_.data);
            }
            else{
                LOG_ERROR("Subtask %s defines task frames %s and %s but at least one of them has not been added to robot model",
                          task_name.c_str(), sub_task->root_.c_str(), sub_task->tip_.c_str());
                throw std::runtime_error("Invalid sub task");
            }
        }
        else{ //Task is in joint space: Task Matrix == Identity
            sub_task->A_ = Eigen::MatrixXd::Identity(sub_task->no_task_variables_, robot_->no_of_joints_);
        }
        break;
    }
    case WBC_TYPE_TORQUE:{
        break;
    }
    default: {
        LOG_ERROR("Invalid WBC type: %i", wbc_type_);
        throw std::invalid_argument("Invalid wbc type");
    }
    }//Switch
}

void Wbc::solve(const WbcInputMap& wbc_input,
                const base::samples::Joints &robot_status,
                base::commands::Joints &solver_output){

    if(!configured_)
        throw std::invalid_argument("Configure has not been called yet");

    if(robot_status.size() != robot_->no_of_joints_){
        LOG_ERROR("Input robot status size is %i but no of robot joints is %i", robot_status.size(), robot_->no_of_joints_);
        throw std::invalid_argument("Invalid number of joints in robot status");
    }
    if(solver_output.size() != robot_->no_of_joints_){
        LOG_WARN("Solver output vector size is %i but total no of robot joints in wbc is %i. Will do a dynamic resize", solver_output.size(), robot_->no_of_joints_);
        solver_output.resize(robot_->no_of_joints_);
    }

    //Update robot model
    robot_->update(robot_status);

    //Update Sub tasks
    for(WbcInputMap::const_iterator it = wbc_input.begin(); it != wbc_input.end(); it++)
        updateSubTask(it->first, it->second);

    //Update equation system
    uint prev_prio = sub_task_vector_[0]->priority_;
    uint prio_index = 0;
    uint row_index = 0;
    uint nq = robot_->no_of_joints_;
    for(uint i = 0; i < sub_task_vector_.size(); i++){

        SubTask* sub_task = sub_task_vector_[i];

        if(sub_task->priority_ > prev_prio){
            prio_index++;
            row_index = 0;
            prev_prio = sub_task->priority_;
        }

        uint nc = sub_task->no_task_variables_;

        y_[prio_index].segment(row_index, nc) = sub_task->y_des_;
        A_[prio_index].block(row_index, 0, nc, nq) = sub_task->A_;

        row_index += sub_task->no_task_variables_;
    }

    //Solve equation system
    solver_->solve(A_, y_, solver_output_);

    //Write output
    switch(wbc_type_){
    case WBC_TYPE_VELOCITY:{
        for(uint i = 0; i < robot_->no_of_joints_; i++)
            solver_output[i].speed = solver_output_(i);
        break;
    }
    default:{
        LOG_ERROR("Invalid WBC type: %i", wbc_type_);
        throw std::invalid_argument("Invalid wbc type");
    }
    }

}

uint Wbc::getNoOfJoints(){
    return robot_->no_of_joints_;
}
