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

}
#endif
