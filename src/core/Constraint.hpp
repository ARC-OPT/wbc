#ifndef TASK_HPP
#define TASK_HPP

#include "TaskConfig.hpp"
#include "RobotModel.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic task for a WBC optimization problem.
 */
class Constraint{
public:

    /** @brief Default constructor */
    Constraint();

    /** @brief Resizes all members */
    Constraint(const TaskConfig& _config, uint n_robot_joints);
    ~Constraint();

    /**
     * @brief Reset task variables to initial values
     */
    void reset();

    /**
     * @brief Update Task matrices and vectors
     */
    virtual void update(RobotModelPtr robot_model) = 0;

    /**
     * @brief Check if the task is in timeout and set the timeout flag accordingly. A task is in timeout if
     *    - No reference value has been set yet
     *    - A timeout value is configured (config.timeout > 0) and no reference value arrived during the timeout period
     */
    void checkTimeout();

    /**
     * @brief Set task weights.
     * @param weights Weight vector. Size has to be same as number of task variables and all entries have to be >= 0
     */
    void setWeights(const base::VectorXd& weights);

    /**
     * @brief Set task activation.
     * @param activation Value has to be between 0 and 1. Can be used to activate(1)/deactivate(0) the task.
     */
    void setActivation(const double activation);

    /** Last time the task reference values was updated.*/
    base::Time time;

    /** Configuration of this task. See TaskConfig.hpp for more details*/
    TaskConfig config;

    /** Reference input for this task. Can be either joint or a Cartesian space variables. If the latter is the case
     *  they have to be expressed in in ref_frame coordinates! See TaskConfig.hpp for more details.*/
    base::VectorXd y_ref;

    /** Reference value for this task. Can be either joint or a Cartesian space variables. In the former case, y_ref_root will be
     *  equal to y_ref, in the latter case, y_ref_root will be y_ref transformed into the robot root/base frame.*/
    base::VectorXd y_ref_root;

    /** Task weights. Size has to be same as number of task variables and all entries have to be >= 0.
     *  A zero entry means that the reference of the corresponding task variable will be ignored while computing the solution, for example
     *  when controlling the Cartesian pose, the last 3 entries can be set to zero in order to ignore the orientarion and only control the position*/
    base::VectorXd weights;

    /** Task weights. In case of joint tasks, weights_root will be equal to weights. In case of Cartesian tasks, weights_root will be equal to
      * weights, transformed into the robot's base/root frame*/
    base::VectorXd weights_root;

    /** Task activation. Has to be between 0 and 1. Will be multiplied with the task weights. Can be used to (smoothly) switch on/off the tasks */
    double activation;

    /** Can be 0 or 1. Will be multiplied with the task weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero*/
    int timeout;

    /** Task matrix */
    base::MatrixXd A;

    /** Weighted task matrix */
    base::MatrixXd Aw;
};
using Task = Constraint;

// to be renamed TaskPtr
typedef std::shared_ptr<Task> ConstraintPtr;

using TaskPtr = ConstraintPtr;

} // namespace wbc

#endif
