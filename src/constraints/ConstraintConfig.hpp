#ifndef CONSTRAINT_CONFIG_HPP
#define CONSTRAINT_CONFIG_HPP

#include <string>
#include <vector>

namespace wbc{

/**
 * Two type of constraints are possible:
 *  - Cartesian constraints: The motion between two coordinate frames (root, tip) will be constrained by a given twist (translational
 *                           and rotational velocity in Cartesian space). This can be used for operational space control, e.g.
 *                           Cartesian force/position control, obstacle avoidance, ...
 *  - Joint constraints: The motion for the given joints will be constrained by a joint space velocity. This can be used for joint space
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
    ConstraintConfig(){
        timeout = 0;
        activation = 0;
    }

    /** Constraint type, can be one of 'jnt' (joint space) or 'cart' (Cartesian) */
    constraint_type type;

    /** Priority of this constraint. 0-based. 0 ^= highest priority */
    int priority;

    /** Unique identifier of the constraint*/
    std::string name;

    /** Only joint space constraints: names of the involved joints */
    std::vector<std::string> joint_names;

    /** Initial weights for this constraint. Entries have to be >= 0. Can be used to balance contributions of the constraint variables.
     *  A value of 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution.
     */
    std::vector<double> weights;

    /** Initial activation for this constraint. Has to be between 0..1. Can be used to enable(1)/disable(0) the whole constraint. Default is zero.*/
    double activation;

    /** Timeout of this constraint in seconds. Output for this constraint will be set to zero if, for more than this amount of time, no new reference is set.
        A value of <= 0 will be interpreted as infinite, which means the task never goes into timeout. Default is zero*/
    double timeout;

    /**
     * Only Cartesian Constraints: Root frame of the kinematic chain associated with this Constraint.
     * Has to be the name of a link available in robot`s KDL tree.
     * This parameter is neglected if the Constraint is defined in joint space
     */
    std::string root;

    /** Only Cartesian constraints: Tip frame of the kinematic chain associated with this constraint.
     *  Has to be the name of a link available in robot`s KDL tree or an empty string.
     *  This parameter is neglected if the Constraint is defined in joint space
     */
    std::string tip;

    /** Only Cartesian constraints: Reference frame of the constraint input (base with respect to which the input twist is expressed).
     *  This has to be the name of a link available in robot`s KDL tree.
     *  If ref_frame==root the input is assumed to be given in root frame. Otherwise it will be transformed to root.
     *  Note that the reference point of the input velocity will still be ref_frame. This means that the rotational velocity will
     *  describe a rotation around the origin of ref_frame, but expressed with respect to the robot base.
     */
    std::string ref_frame;
};

}
#endif
