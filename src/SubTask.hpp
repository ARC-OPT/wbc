#include <Eigen/Core>
#include <kdl/jacobian.hpp>
#include "SubTaskConfig.hpp"

namespace wbc{

/**
 * @brief helper class to carry sub task specific information
 */
class SubTask{
public:
    SubTask(const SubTaskConfig& config, const uint no_robot_joints){
        config_ = config;
        uint no_task_vars;
        if(config.type == task_type_cartesian)
            no_task_vars = 6;
        else
            no_task_vars = config.joints.size();

        y_des_.resize(no_task_vars);

        task_weights_.resize(no_task_vars);
        task_weights_.setConstant(1);

        task_jac_ = KDL::Jacobian(no_task_vars);
        task_jac_.data.setZero();

        H_.resize(no_task_vars,6);
        H_.setZero();

        Uf_.resize(6, no_task_vars);
        Uf_.setIdentity();

        Vf_.resize(no_task_vars, no_task_vars);
        Vf_.setIdentity();

        Sf_.resize(no_task_vars);
        Sf_.setZero();

        A_.resize(no_task_vars, no_robot_joints);
        A_.setZero();
    }

    ~SubTask(){}

    SubTaskConfig config_;

    Eigen::VectorXd y_des_;
    Eigen::VectorXd task_weights_;
    KDL::Jacobian task_jac_;
    KDL::Frame pose_;
    Eigen::MatrixXd A_;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf_, Vf_;
    Eigen::VectorXd Sf_;
    Eigen::MatrixXd H_;
};

}
