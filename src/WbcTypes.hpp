#ifndef WBC_TYPES_HPP
#define WBC_TYPES_HPP

#include <kdl/jacobian.hpp>
#include <vector>

namespace wbc{

enum task_type{task_type_joint, task_type_cartesian};


/**
 * @brief Defines a sub task in the whole body control problem. Valid Configuration are e.g.
 *        - task_type = wbc::joint
 *          root_frame = "Robot_base"
 *          tip_frame = "Gripper"
 *
 *        - task_type = WBC_TASK_TYPE_JOINT
 *          joints_ = ["J_1", "J_2", "J_3"]
 */
class SubTaskConfig{
public:
    /** Whole body task type, can be joint space or Cartesian for now */
    task_type type;

    /** Unique identifier of the task*/
    std::string name;

    /**
     * Only Cartesian Tasks: Root frame associated with this task.
     * Has to be the name of a link available in robot`s KDL tree.
     * This parameter is neglected if the task is in joint space
     */
    std::string root;

    /** Only Cartesian Tasks: Tip frame associated with this task.
     *  Has to be the name of a link available in robot`s KDL tree or an empty string.
     *  If empty, the task is assumed to in joint space
     */
    std::string tip;

    /** Only Joint Space Tasks: In case the task is of type WBC_TASK_TYPE_JOINT,
     * the joints used for this task have to be specified here.
     */
    std::vector<std::string> joints;

    /** Priority of this subtask. 0-based. 0 ^= highest priority */
    uint priority;
};
}
#endif
