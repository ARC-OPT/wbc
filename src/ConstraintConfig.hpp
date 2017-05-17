#ifndef CONSTRAINT_CONFIG_HPP
#define CONSTRAINT_CONFIG_HPP

#include <string>
#include <vector>
#include <stdexcept>

namespace wbc{

/**
 * Two type of constraints are possible:
 *  - Cartesian constraints: The motion between two coordinate frames (root, tip) will be constrained. This can be used for operational space control, e.g.
 *                           Cartesian force/position control, obstacle avoidance, ...
 *  - Joint constraints: The motion for the given joints will be constrained. This can be used for joint space
 *                       control, e.g. avoiding the joint limits, maintaining a certain elbow position, joint position control, ...
 */
enum constraint_type{jnt, cart};

/**
 * @brief Defines a constraint in the whole body control problem. Valid Configuration are e.g.
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
class ConstraintConfig{

public:
    ConstraintConfig() :
        activation(0),
        timeout(0){}

    /** Unique identifier of the constraint. Must not be empty*/
    std::string name;

    /** Constraint type, can be one of 'jnt' (joint space) or 'cart' (Cartesian) */
    constraint_type type;

    /** Priority of this constraint. Must be >= 0! 0 ^= highest priority. */
    int priority;

    /** Initial weights for this constraint. Entries have to be >= 0. Can be used to balance contributions of the constraint variables.
     *  A value of 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution.
     *  Vector Size has to be same as number of constraint variables. e.g. number of joint names in case of joint space constraint,
        and 6 in case of a Cartesian Constraint */
    std::vector<double> weights;

    /** Initial activation for this constraint. Has to be within 0 and 1. Can be used to enable(1)/disable(0) the whole constraint,
     *  or to apply a smooth activation function. Default is 0.*/
    double activation;

    /** Timeout of this constraint in seconds. Output for this constraint will be set to zero if, for more than this amount of time, no new reference is set.
        A value of <= 0 will be interpreted as infinite, which means the task never goes into timeout. Default is zero*/
    double timeout;

    /** Only joint constraints: names of the involved joints. Must not be empty */
    std::vector<std::string> joint_names;

    /** Only Cartesian Constraints: Root frame of the kinematic chain associated with this Constraint.
     *  Has to be the name of a link available in the robot model. */
    std::string root;

    /** Only Cartesian constraints: Tip frame of the kinematic chain associated with this constraint.
     *  Has to be the name of a link available in the robot model */
    std::string tip;

    /** Only Cartesian constraints: Reference frame of the constraint input (base with respect to which the input is expressed).
     *  This has to be the name of a link available in robot model.
     *  If ref_frame == root the input is assumed to be given in root frame. Otherwise it will be transformed to root. */
    std::string ref_frame;

    /** Check for valid entries of the constraint config */
    void validate() const{
        if(name.empty())
            throw std::invalid_argument("ConstraintConfig: Constraint name must not be empty!");
        if(activation < 0 || activation > 1)
            throw std::invalid_argument("Constraint " + name + ": Activation must between 0 and 1, but is " + std::to_string(activation));
        if(priority < 0)
            throw std::invalid_argument("Constraint " + name + ": Priority must >= 0, but is " + std::to_string(priority));
        if(type == cart){
            if(root.empty())
                throw std::invalid_argument("Constraint " + name + ": Root frame is empty");
            if(tip.empty())
                throw std::invalid_argument("Constraint " + name + ": Tip frame is empty");
            if(ref_frame.empty())
                throw std::invalid_argument("Constraint " + name + ": Ref frame is empty");
            if(weights.size() != 6)
                throw std::invalid_argument("Constraint " + name + ": Size of weight vector should be 6, but is " + std::to_string(weights.size()));
        }
        else if(type == jnt){
            if(weights.size() != joint_names.size())
                throw std::invalid_argument("Constraint " + name + ": Size of weight vector should be " + std::to_string(joint_names.size()) + ", but is " + std::to_string(weights.size()));
            for(size_t i = 0; i < joint_names.size(); i++)
                if(joint_names[i].empty())
                    throw std::invalid_argument("Constraint " + name + ": Name of joint " + std::to_string(i) + " is empty!");
        }
        else
            throw std::invalid_argument("Constraint " + name + ": Invalid constraint type: " + std::to_string(type));

        for(size_t i = 0; i < weights.size(); i++)
            if(weights[i] < 0)
                throw std::invalid_argument("Constraint " + name + ": Constraint weights must be >= 0, but weight " + std::to_string(i) + " is " + std::to_string(weights[i]));
    }

    uint noOfConstraintVariables() const{
        if(type == cart)
            return 6;
        else
            return joint_names.size();
    }
};

}
#endif
