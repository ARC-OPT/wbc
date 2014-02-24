#ifndef WBC_HPP
#define WBC_HPP

#include "SubTaskConfig.hpp"
#include <kdl/tree.hpp>
#include <base/commands/joints.h>
#include <Eigen/Core>

namespace wbc{

class TaskFrame;
class SubTask;

typedef std::map<std::string, SubTask*> SubTaskMap;
typedef std::map<std::string, int> JointIndexMap;
typedef std::map<std::string, TaskFrame*> TaskFrameMap;

class WbcVelocity{
protected:
    bool addTaskFrame(const std::string &frame_id);
    void clear();

    TaskFrameMap task_frame_map_;

    bool configured_;
    KDL::Tree tree_;
    std::string robot_root_;

    Eigen::VectorXd solver_output_; /** Control solution. Size: No of joints in kdl tree. Order of joints will be same as in status vector given in solve() */

    //Helpers
    Eigen::VectorXd temp_;

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
     * @param config Sub task configuration vector.
     * @return True in case of success, false else
     */
    bool configure(const KDL::Tree tree,
                   const std::vector<SubTaskConfig> &config,
                   const std::vector<std::string> &joint_names);

    SubTask* subTask(const std::string &name);

    void update(const base::samples::Joints &status);

    std::vector<std::string> jointNames();

    std::vector<Eigen::MatrixXd> A_; /** Vector of task matrices per priority. These define, together with y, for each priority the linear equation system that has to be solved */
    std::vector<Eigen::VectorXd> y_ref_; /** Vector of desired task variables per priority */
    std::vector<Eigen::MatrixXd> Wy_; /** Vector of task weights per priority */
    uint no_robot_joints_;
    std::vector<uint> no_task_vars_pp_;
    JointIndexMap joint_index_map_;
    std::map<std::string, SubTask*> sub_task_map_; /** Map associating names of subtasks to subtask pointers */
    std::vector< std::vector<SubTask*> > sub_task_vector_; /** Vector containing all sub tasks (ordered by priority, highest priority first) */

};
}
#endif

