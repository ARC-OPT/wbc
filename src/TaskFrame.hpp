#ifndef TASK_FRAME_HPP
#define TASK_FRAME_HPP

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_conversions/KDLConversions.hpp>

namespace wbc{

class TaskFrame{
public:
    TaskFrame(){}
    TaskFrame(const uint nx, const std::string &name){
        jac.resize(6,nx);
        jac.setZero();
        tf_name = name;
    }
    base::Time time;
    std::string tf_name;
    base::MatrixXd jac;
    base::samples::RigidBodyState pose;
};

class TaskFrameKDL{
public:
    TaskFrameKDL(){}
    TaskFrameKDL(const uint nx, const std::string &name){
        jac = KDL::Jacobian(nx);
        jac.data.setZero();
        tf_name = name;
    }
    std::string tf_name;
    KDL::Jacobian jac;
    KDL::Frame pose;
};

void TfToTfKDL(const TaskFrame &tf, TaskFrameKDL& tf_kdl);
void TfKDLToTf(const TaskFrameKDL& tf_kdl, TaskFrame &tf );

}
#endif
