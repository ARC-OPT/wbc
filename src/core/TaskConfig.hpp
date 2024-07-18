#ifndef WBC_CORE_TASK_CONFIG_HPP
#define WBC_CORE_TASK_CONFIG_HPP

#include <string>
#include <vector>
#include <stdexcept>

namespace wbc{

enum TaskType{
    unset = -1,
    spatial_velocity,
    spatial_acceleration,
    com_velocity,
    com_acceleration,
    joint_velocity,
    joint_acceleration,
    wrench_forward
};

/**
 * @brief Defines a task in the whole body control problem.
 */
class TaskConfig {

public:
    TaskConfig(){}
    /** Default constructor for Cartesian space tasks*/
    TaskConfig(const std::string &name,
               const int priority,
               const std::vector<double> weights,
               const double activation);
    ~TaskConfig();

    /** Unique identifier of the constraint. Must not be empty*/
    std::string name;

    /** Priority of this task. Must be >= 0! 0 corresponds to the highest priority. Currently only one priority level is supported */
    int priority;

    /** Initial weights for this task. Size has to be same as number of task variables. Entries have to be >= 0.
     *  Can be used to balance contributions of the task variables.
     *  A value of 0 means that the reference of the corresponding task variable will be ignored while computing the solution.
     *  Vector Size has to be same as number of task variables. e.g. number of joint names in case of joint space task,
        and 6 in case of a Cartesian task */
    std::vector<double> weights;

    /** Initial activation for this task. Has to be within 0 and 1. Can be used to enable(1)/disable(0) the whole task,
     *  or to apply a smooth activation function. Default is 0.*/
    double activation;

    /** Check for valid entries of the task config */
    bool isValid() const;
};

}

#endif // WBC_CORE_TASK_CONFIG_HPP
