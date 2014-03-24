#ifndef EXTENDED_SUB_TASK_HPP
#define EXTENDED_SUB_TASK_HPP

#include "SubTask.hpp"

#include <kdl/jacobian.hpp>

namespace wbc{

class ExtendedSubTask : public SubTask{
public:
    ExtendedSubTask(){}
    ExtendedSubTask(const SubTaskConfig& _config,
                    const uint _no_robot_joints,
                    bool _tasks_active  = true) :
        SubTask(_config, _no_robot_joints, _tasks_active)
    {
        task_jac = KDL::Jacobian(no_task_vars);
        task_jac.data.setZero();

        H.resize(no_task_vars,6);
        H.setZero();

        Uf.resize(6, no_task_vars);
        Uf.setIdentity();

        Vf.resize(no_task_vars, no_task_vars);
        Vf.setIdentity();

        Sf.resize(no_task_vars);
        Sf.setZero();

    }

    KDL::Jacobian task_jac;
    KDL::Frame pose;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf, Vf;
    Eigen::VectorXd Sf;
    Eigen::MatrixXd H;
};
}

#endif
