#ifndef WBC_HPP
#define WBC_HPP

#include "SubTaskConfig.hpp"
#include <kdl/tree.hpp>
#include <base/commands/joints.h>
#include <Eigen/Core>

namespace wbc{

class TaskFrame;
class ExtendedSubTask;
class SubTask;
class HierarchicalWDLSSolver;

typedef std::map<std::string, ExtendedSubTask*> SubTaskMap;
typedef std::map<std::string, int> JointIndexMap;
typedef std::map<std::string, TaskFrame*> TaskFrameMap;

class WbcVelocity{
protected:
    bool addTaskFrame(const std::string &frame_id);
    void clear();

    HierarchicalWDLSSolver* solver_;

    bool debug_;
    bool configured_;
    KDL::Tree tree_;
    std::string robot_root_;
    SubTaskMap sub_task_map_; /** Map associating names of subtasks to subtask pointers */
    std::vector< std::vector<ExtendedSubTask*> > sub_task_vector_;  /** Vector containing all sub tasks (ordered by priority, highest priority first) */
    TaskFrameMap task_frame_map_; /** Map containing all task frames, that are associated with the cartesian tasks*/
    double task_timeout_; /** In seconds. A task will be deactivated if no new task reference comes in for such a long time. If set to .nan, no timeout will be used. */
    uint no_robot_joints_;
    JointIndexMap joint_index_map_; /** Maps joint names to indices. Order will be either as in KDL::Tree, or as in joint_names parameter given to configure() */

    //Helpers
    Eigen::VectorXd temp_;
    bool has_timeout_;
    KDL::Twist tw_;

    std::vector<Eigen::MatrixXd> A_; /** Vector of task matrices per priority. These define, together with y, for each priority the linear equation system that has to be solved */
    std::vector<Eigen::VectorXd> y_ref_; /** Vector of desired task variables per priority */
    std::vector<Eigen::MatrixXd> Wy_; /** Vector of task weights per priority */
public:
    /**
     * @brief Create Robot Model and Solver
     * @param tree Kinematic tree for the robot model
     * @param type Which wbc to use?
     */
    WbcVelocity();
    ~WbcVelocity();

    /**
     * @brief Configure Has to be called before calling solve for the first time.
     * @param tree KDL Tree describing the robot
     * @param config Sub task configuration vector.
     * @param joint_names Optional order of joint names that shall be used internally. If empty the order from KDL::Tree will be used (alphabetic)
     * @param tasks_active Tasks will be active in the beginning, if this is set to true
     * @param task_timeout In seconds. Tasks will be timed out after this amount of time, their weights and inout will be zero, so that they do
     *                     not disturb the active tasks anymore. If task_timeout == .nan no timeout will be used.
     * @return True in case of success, false else
     */
    bool configure(const KDL::Tree tree,
                   const std::vector<SubTaskConfig> &config,
                   const std::vector<std::string> &joint_names,
                   bool tasks_active,
                   double task_timeout,
                   bool debug = false);

    /**
     * @brief solve Compute control solution given all the constraints
     * @param status Joint status. Has to contain at least all joints that have been configured, i.e. all joints that are located between the robot root from the given KDL::Tree and
     *               and all task frames. Order may be arbitrary. The joints will be mapped correctly according to the given names.
     * @param ctrl_out Control output. Size will be same as total no of joints as returned by noOfJoints()
     */
    void solve(const base::samples::Joints &status, Eigen::VectorXd &ctrl_out);

    uint noOfJoints(){return no_robot_joints_;}
    SubTask* subTask(const std::string &name);
    HierarchicalWDLSSolver* solver(){return solver_;}
    std::vector<std::string> jointNames();
    uint jointIndex(const std::string &joint_name){return joint_index_map_[joint_name];}

};
}
#endif

