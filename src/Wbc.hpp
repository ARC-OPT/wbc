#ifndef WBC_HPP
#define WBC_HPP

#include "WbcTypes.hpp"
#include <kdl/tree.hpp>
#include <base/commands/joints.h>

class HierarchicalSolver;
class RobotModel;

class Wbc{
protected:
    WBC_TYPE wbc_type_; /** Which type of wbc is being used?*/

    RobotModel *robot_; /** Instance of the KDL robot model. Created at construction time of wbc. Contains task frames, Jacobians, etc...*/
    HierarchicalSolver* solver_; /** Solver instance. Created at construction time of wbc */

    std::vector< std::vector<SubTask*> > sub_task_vector_; /** Vector containing all sub tasks (ordered by priority, highest priority first) */

    Eigen::VectorXd solver_output_; /** Control solution. Size: No of joints in kdl tree. Order will be as in status vector given in updateRobotModel */

    std::vector<Eigen::MatrixXd> A_; /** Vector of task matrices per priority. These define, together with y, for each priority the linear equation system that has to be solved */
    std::vector<Eigen::VectorXd> y_; /** Vector of desired task variables per priority */
    std::vector<Eigen::VectorXd> Wy_; /** Vector of task weights per priority */

    bool configured_; /** false in case AddSubTask() has been called without subsequent call of configure() */

    //Helpers
    Eigen::VectorXd temp_;

    void updateSubTask(SubTask* sub_task);

public:
    /**
     * @brief Create Robot Model and Solver
     * @param tree Kinematic tree for the robot model
     * @param type Which wbc to use?
     */
    Wbc(KDL::Tree tree, const WBC_TYPE type);
    ~Wbc();

    /**
     * @brief Configure Has to be called before calling solve for the first time.
     * @param config Sub task configuration vector.
     * @return True in case of success, false else
     */
    bool configure(const WbcConfig &config);

    /**
     * @brief solve
     * @param task_ref
     * @param task_weights
     * @param joint_weights
     * @param robot_status
     * @param solver_output
     */
    void solve(const WbcInput& task_ref,
               const WbcInput& task_weights,
               const Eigen::VectorXd joint_weights,
               const base::samples::Joints &robot_status,
               base::commands::Joints &solver_output);

    uint getNoOfJoints();
    HierarchicalSolver* solver(){return solver_;}
    RobotModel* robot(){return robot_;}
};

#endif

