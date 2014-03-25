#ifndef SUB_TASK_HPP
#define SUB_TASK_HPP

#include <base/Eigen.hpp>
#include "SubTaskConfig.hpp"
#include <base/time.h>

namespace wbc{

/**
 * @brief helper class to carry sub task specific information
 */
class SubTask{
public:
    SubTask(){}
    SubTask(const SubTaskConfig& _config,
            const uint _no_robot_joints,
            bool _tasks_active  = true){

        config = _config;

        if(config.type == task_type_cartesian)
            no_task_vars = 6;
        else
            no_task_vars = config.joint_names.size();

        y_des.resize(no_task_vars);
        y_des.setZero();

        y_solution.resize(no_task_vars);
        y_solution.setZero();

        y.resize(no_task_vars);
        y.setZero();

        y_des_root_frame.resize(no_task_vars);
        y_des_root_frame.setZero();

        weights = base::VectorXd::Ones(no_task_vars);

        A.resize(no_task_vars, _no_robot_joints);
        A.setZero();

        task_timed_out = 0;
        if(_tasks_active)
            activation = 1;
        else
            activation = 0;
    }

    SubTaskConfig config;

    base::VectorXd y_des;
    base::VectorXd y_des_root_frame;
    base::VectorXd y_solution;
    base::VectorXd y;
    base::VectorXd weights;
    double activation;
    int task_timed_out;

    base::MatrixXd A;
    uint no_task_vars;
    base::Time last_task_input;
};

}
#endif
