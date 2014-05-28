#ifndef WBC_TYPES_HPP
#define WBC_TYPES_HPP

#include <string>
#include <vector>

namespace wbc{

enum task_type{jnt, cart};
enum task_ref_frame{task_ref_frame_root, task_ref_frame_tip};


/**
 * @brief Defines a sub task in the whole body control problem. Valid Configuration are e.g.
 *        - task_type = cart
 *          name = "bla"
 *          priority = 0
 *          root_frame = "Robot_base"
 *          tip_frame = "Gripper"
 *
 *        - name = "bla"
 *          priority = 1
 *          task_type = jnt
 *          joint_names = ["J_1", "J_2", "J_3"]
 */
class SubTaskConfig{
public:
    SubTaskConfig(){}
    SubTaskConfig(const task_type _type,
                  const int _priority,
                  const std::string& _name,
                  const std::vector<std::string>& _task_var_names,
                  const std::string &_root = "",
                  const std::string &_tip = "",
                  const task_ref_frame& _ref_frame = task_ref_frame_root) :
        type(_type),
        priority(_priority),
        name(_name),
        task_var_names(_task_var_names),
        root(_root),
        tip(_tip),
        ref_frame(_ref_frame){}


    /** Whole body task type, can be joint space or Cartesian for now */
    task_type type;

    /** Priority of this subtask. 0-based. 0 ^= highest priority */
    int priority;

    /** Unique identifier of the task*/
    std::string name;

    /** Names of the task variables. Joint names or names of the Cartesian directions */
    std::vector<std::string> task_var_names;

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

    /** Only Cartesian tasks: Reference frame. If reference frame is choosen to be tip, the input will be converted to root internally. */
    task_ref_frame ref_frame;
};

}
#endif
