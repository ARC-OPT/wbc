#ifndef TASK_STATUS_HPP
#define TASK_STATUS_HPP

#include "Task.hpp"

namespace wbc {

// to be renamed Task Status

class TaskStatus{
public:
    /** Last time the task reference values was updated.*/
    base::Time time;

    /** Configuration of this task. See TaskConfig.hpp for more details.*/
    TaskConfig config;

    /** Task activation. Has to be between 0 and 1. Will be multiplied with the task weights. Can be used to (smoothly) switch on/off the tasks.*/
    double activation;

    /** Can be 0 or 1. Will be multiplied with the task weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero.*/
    int timeout;

    /** Task weights in root coordinates.*/
    base::VectorXd weights;

    /** Reference input for this task in root coordinates. Can be either joint or a Cartesian space variables.*/
    base::VectorXd y_ref;

    /** Solution as computed by the solver for this task in root coordinates.*/
    base::VectorXd y_solution;

    /** Actual task as executed on the robot. For Cartesian tasks, this will be back transformed to
     *  Cartesian space and defined in root coordinates.*/
    base::VectorXd y;
};


class TasksStatus : public base::NamedVector<TaskStatus>{
};

}

#endif
