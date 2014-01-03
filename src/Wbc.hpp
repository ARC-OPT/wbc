#ifndef WBC_HPP
#define WBC_HPP

#include <kdl/tree.hpp>
#include <base/commands/joints.h>
#include <Eigen/Core>

class HierarchicalSolver;
class SubTask;
class RobotModel;

typedef std::map<std::string, SubTask*> SubTaskMap;
typedef std::map<uint, uint> PriorityMap;
typedef std::map<std::string, Eigen::VectorXd> WbcInputMap;

enum WBC_TYPE{WBC_TYPE_VELOCITY, WBC_TYPE_TORQUE};

class Wbc{
protected:
    WBC_TYPE wbc_type_; /** Which type of wbc is being used?*/

    RobotModel *robot_; /** Instance of the KDL robot model. Created at construction time of wbc. Contains task frames, Jacobians, etc...*/
    HierarchicalSolver* solver_; /** Solver instance. Created at construction time of wbc */

    std::vector<SubTask*> sub_task_vector_; /** Vector containing all sub tasks (order by priority, highest priority first) */
    SubTaskMap sub_task_map_; /** Maps names of sub tasks on Sub task pointers */
    PriorityMap priority_map_; /** Maps prioriteis to number of task variables in this priority level */

    Eigen::VectorXd solver_output_; /** Control solution. Size: No of joints in kdl tree. Order will be as in status vector given in updateRobotModel */
    std::vector<Eigen::MatrixXd> A_; /** Vector of task matrices per priority. These define, together with y, for each priority the linear equation system that has to be solved */
    std::vector<Eigen::VectorXd> y_; /** Vector of desired task variables per priority */

    bool configured_; /** false in case AddSubTask() has been called without subsequent call of configure() */

    //Helpers
    Eigen::VectorXd temp_;

    void updateSubTask(const std::string& task_name,
                       const Eigen::VectorXd &y_des);

public:
    /**
     * @brief Create Robot Model and Solver
     * @param tree Kinematic tree for the robot model
     * @param type Which wbc to use?
     */
    Wbc(KDL::Tree tree, const WBC_TYPE type);
    ~Wbc();

    /**
     * @brief AddSubTask Add sub task to wbc
     * @param task_name Unique identifier of sub task
     * @param priority Priority of the task. Tasks with lower priority will be solved in the null space projected from the higher priorities. 0 corresponds to highest priority
     * @param root Root frame associated with this task. Has to be the name of a link available in robot`s KDL tree or an empty string. If empty, the task is assumed to in joint space
     * @param tip Tip frame associated with this task. Has to be the name of a link available in robot`s KDL tree or an empty string. If empty, the task is assumed to in joint space
     * @param no_task_variables No of variables associated with this task. E.g. a 6D Cartesian task would have 6 variables. This parameter is neglected if the task is in joint space
     * @return True in case of success, false otherwise
     */
    bool addSubTask(const std::string &task_name,
                    const uint priority,
                    const std::string& root,
                    const std::string& tip,
                    const uint no_task_variables);

    /**
     * @brief Configure Has to be called after all sub tasks have been added
     * @return True in case of success, false else
     */
    bool configure();

    /**
     * @brief updateSubTask Update the desired Sub task output
     * @param task_name
     * @param y_des
     */
    /**
     * @brief solve Solve wbc problem for current robot status wrt given wbc input
     * @param wbc_input Map that maps task names on task variables (constraints). E.g. name="CartesianPose", task variables=(x,y,z, rot_x,rot_y,rot_z).
     * @param robot_status Current position, velocity and acceleration of all robot joints in given kdl tree.
     * @param solver_output Control solution that satisfies the constraints defined by wbc input. Size will be equal to the number of all robot joints in kdl tree.
     *                      Order will be equal to the order in robot_status
     */
    void solve(const WbcInputMap& wbc_input,
               const base::samples::Joints &robot_status,
               base::commands::Joints &solver_output);

    uint getNoOfJoints();
};

#endif

