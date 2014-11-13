#ifndef TASK_FRAME_KDL_HPP
#define TASK_FRAME_KDL_HPP

#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <vector>

namespace wbc{

class TaskFrameKDL{
public:
    TaskFrameKDL(){}
    TaskFrameKDL(const std::string &name, const std::vector<std::string>& jt_names){
        jac_ = KDL::Jacobian(jt_names.size());
        jac_.data.setZero();
        tf_name_ = name;
        joint_names_ = jt_names;
        jnt_inertia_.resize(jt_names.size(),jt_names.size());
        jnt_inertia_.setZero();
        jnt_gravity_.resize(jt_names.size());
        jnt_gravity_.setZero();
        jnt_coriolis_.resize(jt_names.size());
        jnt_coriolis_.setZero();

    }
    std::string tf_name_;
    KDL::Jacobian jac_;
    std::vector<std::string> joint_names_;
    KDL::Frame pose_;

    Eigen::MatrixXd jnt_inertia_;
    Eigen::VectorXd jnt_gravity_;
    Eigen::VectorXd jnt_coriolis_;
};

}
#endif
