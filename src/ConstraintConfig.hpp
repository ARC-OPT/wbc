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
    ConstraintConfig(){}
    ConstraintConfig(const constraint_type _type,
                  const int _priority,
                  const std::string& _name,
                  const std::string &_root = "",
                  const std::string &_tip = "",
                  const std::vector<std::string>& _joint_names = std::vector<std::string>(),
                  const constraint_ref_frame& _ref_frame = constraint_ref_frame_root) :
        type(_type),
        priority(_priority),
        name(_name),
        joint_names(_joint_names),
        root(_root),
        tip(_tip),
        ref_frame(_ref_frame){}


    /** Constraint type, can be one of 'jnt' (joint space) or 'cart' (Cartesian) */
    constraint_type type;

    /** Priority of this constraint. 0-based. 0 ^= highest priority */
    int priority;

    /** Unique identifier of the constraint*/
    std::string name;

    /** Only joint space constraints: names of the involved joints */
    std::vector<std::string> joint_names;

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

    /** Only Cartesian constraints: Reference frame. If reference frame is choosen to be tip, the input will be converted to root internally. */
    constraint_ref_frame ref_frame;
};

}
#endif
