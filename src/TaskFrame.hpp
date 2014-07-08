#ifndef TASK_FRAME_HPP
#define TASK_FRAME_HPP

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class TaskFrame{
public:
    TaskFrame(){}
    TaskFrame(const uint nx, const std::string &name){
        jac.resize(6,nx);
        jac.setZero();
    }
    base::Time time;
    std::string tf_name;
    base::MatrixXd jac;
    base::samples::RigidBodyState pose;
};

}
#endif
