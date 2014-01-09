#include "Wbc.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>

using namespace std;

namespace wbc{

Wbc::Wbc(KDL::Tree tree){
    configured_ = false;
    robot_ = new RobotModel(tree);
    temp_ = Eigen::VectorXd(6);
}

Wbc::~Wbc(){
    for(uint i = 0; i < sub_task_vector_.size(); i++ ){
        for(uint j = 0; j < sub_task_vector_[i].size(); j++)
            delete sub_task_vector_[i][j];
    }

    sub_task_vector_.clear();
    delete robot_;
}

bool Wbc::configure(const WbcConfig &config){

    if(config.empty())
        throw std::runtime_error("Empty sub task vector");

    sub_task_vector_.resize(config.size());
    std::vector<uint> ny_per_prio(config.size());

    //Walk through all priorities
    for(uint prio = 0; prio < config.size(); prio++ ){

        uint no_task_vars_prio = 0;
        //Walk through all sub tasks of this priority
        for(uint i = 0; i < config[prio].size(); i++){

            uint no_task_vars;
            SubTaskConfig sub_task_conf = config[prio][i];
            if(sub_task_conf.type == cartesian){
                no_task_vars = 6;

                if(!robot_->addTaskFrame(sub_task_conf.root))
                    return false;
                if(!robot_->addTaskFrame(sub_task_conf.tip))
                    return false;
            }
            else
                no_task_vars = sub_task_conf.joints.size();

            SubTask* sub_task = new SubTask(sub_task_conf, robot_->no_of_joints_);
            sub_task_vector_[prio].push_back(sub_task);
            no_task_vars_prio += no_task_vars;
        }

        Eigen::MatrixXd A(no_task_vars_prio, robot_->no_of_joints_);
        Eigen::VectorXd y(no_task_vars_prio);
        Eigen::VectorXd y_act(no_task_vars_prio);
        Eigen::VectorXd Wy(no_task_vars_prio);
        ny_per_prio[prio] = no_task_vars_prio;

        Wy.setConstant(1); //Set all task weights to 1 in the beginning
        A.setZero();
        y.setZero();
        y_act.setZero();

        A_.push_back(A);
        y_ref_.push_back(y);
        y_.push_back(y_act);
        Wy_.push_back(Wy);
    }

    if(!solver_.configure(ny_per_prio, robot_->no_of_joints_))
        return false;

    solver_output_.resize(robot_->no_of_joints_);
    configured_ = true;
    return true;
}


void Wbc::updateSubTask(SubTask* sub_task){

    if(sub_task->config_.type == cartesian){
        uint nc = 6; //Task is Cartesian: always 6 task variables

        TaskFrame* tf_root = robot_->getTaskFrame(sub_task->config_.root);
        TaskFrame* tf_tip = robot_->getTaskFrame(sub_task->config_.tip);

        if(tf_root && tf_tip){ //Task is in Cartesian space
            sub_task->pose_ = tf_root->pose_.Inverse() * tf_tip->pose_;
            sub_task->task_jac_.data.setIdentity();
            sub_task->task_jac_.changeRefPoint(-sub_task->pose_.p);
            sub_task->task_jac_.changeRefFrame(tf_root->pose_);

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
            sub_task->A_ = sub_task->H_.block(0, 0, nc, 6) * tf_tip->jac_robot_.data
                    -(sub_task->H_.block(0, 0, nc, 6) * tf_root->jac_robot_.data);
        }
        else{
            LOG_ERROR("Subtask defines task frames %s and %s but at least one of them has not been added to robot model",
                      sub_task->config_.root.c_str(), sub_task->config_.tip.c_str());
            throw std::runtime_error("Invalid sub task");
        }
    }
    else if(sub_task->config_.type == joint){

        sub_task->A_ = Eigen::MatrixXd::Zero(sub_task->config_.joints.size(), robot_->no_of_joints_);

        for(uint i = 0; i < sub_task->config_.joints.size(); i++){
            std::string joint_name = sub_task->config_.joints[i];
            uint idx = robot_->joint_index_map_[joint_name];
            sub_task->A_(i,idx) = 1.0;
        }
    }
}

void Wbc::solve(const WbcInput& task_ref,
                const WbcInput& task_weights,
                const base::VectorXd joint_weights,
                const base::samples::Joints &robot_status,
                base::samples::Joints &solver_output){

    if(!configured_)
        throw std::invalid_argument("Configure has not been called yet");

    //Check valid input
    uint nq_robot = robot_->no_of_joints_;
    if(task_ref.size() != sub_task_vector_.size() ||
            task_weights.size() != sub_task_vector_.size()){
        LOG_ERROR("Invalid input vector size: Ref size: %i, task Weight size: %i, No of priorities in wbc: %i",
                  task_ref.size(), task_weights.size(), sub_task_vector_.size());
        throw std::invalid_argument("Invalid input vector size");
    }
    if(joint_weights.size() != nq_robot){
        LOG_ERROR("Invalid input vector size: No of robot joints: %i, no of joint weights: %i",
                  robot_->no_of_joints_, joint_weights.size());
        throw std::invalid_argument("Invalid input vector size");
    }
    if(robot_status.size() != nq_robot){
        LOG_ERROR("Input robot status size. No of joints is %i but no of robot joints is %i",
                  robot_status.size(), robot_->no_of_joints_);
        throw std::invalid_argument("Invalid number of joints in robot status");
    }

    if(solver_output.size() != nq_robot){
        LOG_DEBUG("Solver out has size %i but no of robot joints is %i. Will do a resize", solver_output.size(), nq_robot);
        solver_output.resize(nq_robot);
    }

    //Update robot model: This will compute Kinematics and dynamics of all task frames
    robot_->update(robot_status);

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < sub_task_vector_.size(); prio++){

        if(task_ref[prio].size() != sub_task_vector_[prio].size() ||
                task_weights[prio].size() != sub_task_vector_[prio].size()){
            LOG_ERROR("Invalid input vector size: Priority %i defines %i sub tasks, but input size is: Ref: %i, Task Weights: %i",
                      prio, sub_task_vector_[prio].size(), task_ref[prio].size(), task_weights[prio].size());
            throw std::invalid_argument("Invalid input vector size");
        }

        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < sub_task_vector_[prio].size(); i++){

            SubTask *sub_task = sub_task_vector_[prio][i];
            uint no_task_vars;
            if(sub_task->config_.type == cartesian)
                no_task_vars = 6;
            else
                no_task_vars = sub_task->config_.joints.size();

            if(no_task_vars != task_ref[prio][i].rows() ||
                    no_task_vars != task_weights[prio][i].rows()){
                LOG_ERROR("Invalid input vector size: Task %i of Priority %i has %i task variables, but input size is: Ref: %i, Task Weights: %i",
                          i, prio, no_task_vars, task_ref[prio][i].rows(), task_weights[prio][i].rows());
                throw std::invalid_argument("Invalid input vector size");
            }

            //compute equations for single sub task
            updateSubTask(sub_task);

            //insert task equation into equation system of current priority
            Wy_[prio].segment(row_index, no_task_vars) = task_weights[prio][i];
            y_ref_[prio].segment(row_index, no_task_vars) = task_ref[prio][i];
            A_[prio].block(row_index, 0, no_task_vars, nq_robot) = sub_task->A_;

            row_index += no_task_vars;
        }
    }

    solver_.solve(A_, y_ref_, solver_output_);

    for(uint i = 0; i < nq_robot; i++)
        solver_output[i].speed = solver_output_(i);
}

}
