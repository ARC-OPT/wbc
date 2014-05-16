#ifndef SUB_TASK_HPP
#define SUB_TASK_HPP

#include <base/Eigen.hpp>
#include "SubTaskConfig.hpp"
#include <base/time.h>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>
#include <base/logging.h>

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

        tasks_initially_active = _tasks_active;
        config = _config;
        no_task_vars = _config.task_var_names.size();
        y_des.resize(no_task_vars);
        y_solution.resize(no_task_vars);
        y.resize(no_task_vars);
        y_des_root_frame.resize(no_task_vars);
        wbc_error.resize(no_task_vars);
        ctrl_error.resize(no_task_vars);
        weights = base::VectorXd::Ones(no_task_vars);
        A.resize(no_task_vars, _no_robot_joints);

        reset();
    }

    SubTaskConfig config;

    base::VectorXd y_des; /** Reference value for subtask */
    base::VectorXd y_des_root_frame; /** Reference value for subtask, transformed to root frame of subtask */
    base::VectorXd y_solution; /** Solution for this subtask from wbc */
    base::VectorXd wbc_error; /** Error between y_des_root_frame and y_solution (E.g. because of an overdetermined task) */
    base::VectorXd y; /** Actual value of a task (on the real robot)*/
    base::VectorXd ctrl_error; /** Error between y_des_root_frame and y */
    base::VectorXd weights; /** task weights, a 0 means that the reference of the corresponding task variable will be ignored while computing the solution*/
    double activation; /** Between 0 .. 1. Will be multiplied with the task weights. Can be used to (smoothly) switch on/off the tasks */
    int task_timed_out; /** May be 0 or 1. Will be multiplied with the task weights. If no new reference values arrive for a certain time,the task times out*/
    double sqrt_wbc_err; /** Sqrt of sum of squares of wbc_error*/
    double sqrt_ctrl_err; /** Sqrt of sum of squares of ctrl_error*/
    double manipulability; /** Manipulability of the kinematic chain connected with this sub task*/

    base::MatrixXd A; /** Task matrix */
    bool tasks_initially_active;
    uint no_task_vars; /** Number of task variables */
    base::Time last_task_input; /** last time a new reference sample arrived*/
    double update_timediff; /** In seconds. Time difference between two new reference values*/
    double avg_update_rate; /** In Hz. Avg. Update rate with which new reference sample arrived*/

    void setReference(const base::samples::RigidBodyState &ref);
    void setReference(const base::samples::Joints& ref);
    void updateTime();
    void validate();
    void computeDebug(const base::VectorXd& solver_output, const base::VectorXd& robot_vel);
    void reset();
};

}
#endif
