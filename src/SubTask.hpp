#ifndef SUBTASK_HPP
#define SUBTASK_HPP

#include "TaskFrame.hpp"

/**
 * @brief Defines a sub task in the whole body control problem.
 */
class SubTask{
public:
    SubTask(const std::string &root_frame,
            const std::string &tip_frame,
            const uint no_task_variables,
            const uint no_of_joints,
            const uint priority){
        no_task_variables_ = no_task_variables;
        priority_ = priority;
        root_ = root_frame;
        tip_ = tip_frame;

        y_des_.resize(no_task_variables);

        task_jac_ = KDL::Jacobian(no_task_variables);
        task_jac_.data.setZero();

        H_.resize(no_task_variables,6);
        H_.setZero();

        Uf_.resize(6, no_task_variables);
        Uf_.setIdentity();

        Vf_.resize(no_task_variables, no_task_variables);
        Vf_.setIdentity();

        Sf_.resize(no_task_variables);
        Sf_.setZero();

        A_.resize(no_task_variables, no_of_joints);
        A_.setZero();
    }

    ~SubTask(){}

    uint no_task_variables_;
    uint priority_;
    Eigen::VectorXd y_des_;
    std::string root_, tip_;
    KDL::Jacobian task_jac_;
    KDL::Frame pose_;
    Eigen::MatrixXd A_;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf_, Vf_;
    Eigen::VectorXd Sf_;
    Eigen::MatrixXd H_;
};

#endif
