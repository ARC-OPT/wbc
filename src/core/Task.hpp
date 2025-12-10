#ifndef WBC_CORE_TASK_HPP
#define WBC_CORE_TASK_HPP

#include "TaskConfig.hpp"
#include "RobotModel.hpp"
#include <Eigen/Core>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic task for a WBC optimization problem.
 */
class Task {
protected:
    RobotModelPtr robot_model;

public:

    /** @brief Default constructor */
    Task();

    /** @brief Resizes all members
      * @param nc Number of task variables
      * @param nj Number of robot joints
      */
    Task(TaskConfig config, RobotModelPtr robot_model, uint nv, TaskType type);
    ~Task();

    /**
     * @brief Reset task variables to initial values
     */
    void reset();

    /**
     * @brief Update Task matrices and vectors
     */
    virtual void update() = 0;

    /**
     * @brief Set task weights.
     * @param weights Weight vector. Size has to be same as number of task variables and all entries have to be >= 0
     */
    void setWeights(const Eigen::VectorXd& weights);

    /**
     * @brief Set task activation.
     * @param activation Value has to be between 0 and 1. Can be used to activate(1)/deactivate(0) the task.
     */
    void setActivation(const double activation);

    /** Configuration of this task. See TaskConfig.hpp for more details*/
    TaskConfig config;

    /** Reference input for this task. Can be either joint or a Cartesian space variables. In the latter case, the 
     * reference has to be in world coordinates */
    Eigen::VectorXd y_ref;

    /** Task weights. Size has to be same as number of task variables and all entries have to be >= 0.
     *  A zero entry means that the reference of the corresponding task variable will be ignored while computing the solution, for example
     *  when controlling the Cartesian pose, the last 3 entries can be set to zero in order to ignore the orientarion and only control the position*/
    Eigen::VectorXd weights;

    /** Task activation. Has to be between 0 and 1. Will be multiplied with the task weights. Can be used to (smoothly) switch on/off the tasks */
    double activation;

    /** Task matrix */
    Eigen::MatrixXd A;

    /** Weighted task matrix */
    Eigen::MatrixXd Aw;

    /** Number of task variables*/
    uint nv;

    /** number of robot joints*/
    uint nj;

    /** Type of task, see TaskConfig.hpp for details*/
    TaskType type;
};


using TaskPtr = std::shared_ptr<Task> ;

} // namespace wbc

#endif // WBC_CORE_TASK_HPP
