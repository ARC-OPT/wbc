#ifndef TASK_FRAME_HPP
#define TASK_FRAME_HPP

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <vector>

namespace wbc{

class TaskFrame{
public:
    TaskFrame(){}
    TaskFrame(const std::string &name, const std::vector<std::string>& jt_names){
        jac.resize(6,jt_names.size());
        jac.setZero();
        tf_name = name;
        joint_names = jt_names;
    }
    base::Time time;
    std::string tf_name;
    base::MatrixXd jac;
    std::vector<std::string> joint_names;
    base::samples::RigidBodyState pose;
};

class TaskFrameKDL{
public:
    TaskFrameKDL(){}
    TaskFrameKDL(const std::string &name, const std::vector<std::string>& jt_names){
        jac = KDL::Jacobian(jt_names.size());
        jac.data.setZero();
        tf_name = name;
        joint_names = jt_names;
    }
    std::string tf_name;
    KDL::Jacobian jac;
    std::vector<std::string> joint_names;
    KDL::Frame pose;
};

void TfToTfKDL(const TaskFrame &tf, TaskFrameKDL& tf_kdl);
void TfKDLToTf(const TaskFrameKDL& tf_kdl, TaskFrame &tf );

}
#endif
