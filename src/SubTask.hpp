#include <Eigen/Core>
#include <kdl/jacobian.hpp>
#include "SubTaskConfig.hpp"

namespace wbc{

class TaskFrame;

/**
 * @brief helper class to carry sub task specific information
 */
class SubTask{
public:
    SubTask(const SubTaskConfig& config, const uint no_robot_joints,
            TaskFrame* tf_root = 0, TaskFrame* tf_tip = 0){

        config_ = config;
        tf_root_ = tf_root;
        tf_tip_ = tf_tip;

        if(config.type == task_type_cartesian)
            no_task_vars_ = 6;
        else
            no_task_vars_ = config.joints.size();

        y_des_.resize(no_task_vars_);
        y_des_.setZero();

        task_weights_.resize(no_task_vars_, no_task_vars_);
        task_weights_.setIdentity();

        task_jac_ = KDL::Jacobian(no_task_vars_);
        task_jac_.data.setZero();

        H_.resize(no_task_vars_,6);
        H_.setZero();

        Uf_.resize(6, no_task_vars_);
        Uf_.setIdentity();

        Vf_.resize(no_task_vars_, no_task_vars_);
        Vf_.setIdentity();

        Sf_.resize(no_task_vars_);
        Sf_.setZero();

        A_.resize(no_task_vars_, no_robot_joints);
        A_.setZero();
    }

    ~SubTask(){}

    SubTaskConfig config_;

    TaskFrame* tf_root_;
    TaskFrame* tf_tip_;

    Eigen::VectorXd y_des_;
    Eigen::MatrixXd task_weights_;

    KDL::Jacobian task_jac_;
    KDL::Frame pose_;
    Eigen::MatrixXd A_;
    uint no_task_vars_;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf_, Vf_;
    Eigen::VectorXd Sf_;
    Eigen::MatrixXd H_;
};

}
