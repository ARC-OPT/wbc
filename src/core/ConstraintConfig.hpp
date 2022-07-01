#ifndef TASK_CONFIG_HPP
#define TASK_CONFIG_HPP

#include <string>
#include <vector>
#include <stdexcept>

namespace wbc{

/**
 * Task Type. Two types of tasks are possible:
 *  - Cartesian tasks: The motion between two coordinate frames (root, tip) will be constrained. This can be used for operational space control, e.g.
 *                           Cartesian force/position control, obstacle avoidance, ...
 *  - Joint tasks: The motion for the given joints will be constrained. This can be used for joint space
 *                       control, e.g. avoiding the joint limits, maintaining a certain elbow position, joint position control, ...
 */
enum TaskType{unset = -1,
                jnt = 0,
                cart = 1};

using ConstraintType [[deprecated("Renamed to TaskType")]] = TaskType;

/**
 * @brief Defines a task in the whole body control problem. Valid Configurations are e.g.
 *        - constraint_type = cart
 *          name = "cartesian_position_contol"
 *          priority = 0
 *          root_frame = "Robot_base"
 *          tip_frame = "Gripper"
 *          ref_frame = "Robot_base"
 *          activation = 0
 *
 *        - name = "joint_position_control"
 *          priority = 1
 *          constraint_type = jnt
 *          joint_names = ["J_1", "J_2", "J_3"]
 *          activation = 1
 *          timeout = 3.0
 */
class ConstraintConfig {

public:
    ConstraintConfig();
    /** Default constructor for Cartesian space tasks*/
    ConstraintConfig(const std::string &name,
                     const int priority,
                     const std::string root,
                     const std::string tip,
                     const std::string ref_frame,
                     const double activation = 0,
                     const std::vector<double> weights = {1,1,1,1,1,1},
                     const double timeout = 0);
    /** Default constructor for joint space tasks*/
    ConstraintConfig(const std::string &name,
                     const int priority,
                     const std::vector<std::string> joint_names,
                     const std::vector<double> weights,
                     const double activation = 0,
                     const double timeout = 0);
    ~ConstraintConfig();

    /** Unique identifier of the constraint. Must not be empty*/
    std::string name;

    /** Constraint type, can be one of 'jnt' (joint space) or 'cart' (Cartesian) */
    TaskType type;

    /** Priority of this task. Must be >= 0! 0 corresponds to the highest priority. */
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

    /** Timeout of this task in seconds. Output for this task will be set to zero if, for more than this amount of time, no new reference is set.
        A value of <= 0 will be interpreted as infinite, which means the task never goes into timeout. Default is zero*/
    double timeout;

    /** Only joint tasks: names of the involved joints. Must not be empty */
    std::vector<std::string> joint_names;

    /** Only Cartesian tasks: Root frame of the kinematic chain associated with this task.
     *  Has to be the name of a link available in the robot model. */
    std::string root;

    /** Only Cartesian tasks: Tip frame of the kinematic chain associated with this task.
     *  Has to be the name of a link available in the robot model */
    std::string tip;

    /** Only Cartesian tasks: Reference frame of the task input (base with respect to which the input is expressed).
     *  This has to be the name of a link available in robot model.
     *  If ref_frame == root the input is assumed to be given in root frame. Otherwise it will be transformed to root. */
    std::string ref_frame;

    /** Check for valid entries of the task config */
    void validate() const;

    /** Return the number of task variables for this config depending on the task type*/
    unsigned int nVariables() const;
};

using TaskConfig = ConstraintConfig;

}

#endif
