#ifndef CONSTRAINT_CONFIG_HPP
#define CONSTRAINT_CONFIG_HPP

#include <string>
#include <vector>

namespace wbc{

enum constraint_type{jnt, cart};
enum constraint_ref_frame{constraint_ref_frame_root, constraint_ref_frame_tip};


/**
 * @brief Defines a constraint in the whole body control problem. Valid Configuration are e.g.
 *        - constraint_type = cart
 *          name = "bla"
 *          priority = 0
 *          root_frame = "Robot_base"
 *          tip_frame = "Gripper"
 *
 *        - name = "bla"
 *          priority = 1
 *          constraint_type = jnt
 *          joint_names = ["J_1", "J_2", "J_3"]
 */
class ConstraintConfig{
public:

    /** Constraint type, can be one of 'jnt' (joint space) or 'cart' (Cartesian) */
    constraint_type type;

    /** Priority of this constraint. 0-based. 0 ^= highest priority */
    int priority;

    /** Unique identifier of the constraint*/
    std::string name;

    /** Only joint space constraints: names of the involved joints */
    std::vector<std::string> joint_names;

    /** Initial weights for this constraint. Entries have to be >= 0. Can be used to balance contributions of the constraint variables.
     *  A value of 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution
     */
    std::vector<double> weights;

    /** Initial activation for this constraint. Has to be between 0..1. Can be used to enable(1)/disable(0) the whole constraint. */
    double activation;

    /**
     * Only Cartesian Constraints: Root frame associated with this Constraint.
     * Has to be the name of a link available in robot`s KDL tree.
     * This parameter is neglected if the Constraint is in joint space
     */
    std::string root;

    /** Only Cartesian constraints: Tip frame associated with this constraint.
     *  Has to be the name of a link available in robot`s KDL tree or an empty string.
     *  If empty, the constraint is assumed to in joint space
     */
    std::string tip;

    /** Only Cartesian constraints: Reference frame of the constraint input (base to which the input twist is expressed).
     *  If ref_frame==constraint_ref_frame_tip the input will be automatically transformed to the robot root.
     *  Note that the reference point of the input velocity will
     *  still be ref_frame. This means that the rotational velocity will describe a rotation around the origin of ref_frame,
     *  but expressed with respect to the robot base.
     */
    constraint_ref_frame ref_frame;
};

}
#endif
