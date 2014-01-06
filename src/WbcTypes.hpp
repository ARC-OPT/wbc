#ifndef WBC_TYPES_HPP
#define WBC_TYPES_HPP

#include <kdl/jacobian.hpp>
#include <vector>

enum WBC_TYPE{WBC_TYPE_VELOCITY, WBC_TYPE_TORQUE};
enum WBC_TASK_TYPE{WBC_TASK_TYPE_JOINT, WBC_TASK_TYPE_CARTESIAN};

/**
 * @brief Defines a sub task in the whole body control problem. Valid Configuration are e.g.
 *        - task_type = WBC_TASK_TYPE_CARTESIAN
 *          no_task_variables = 6
 *          root_frame = "Robot_base"
 *          tip_frame = "Gripper"
 *
 *        - task_type = WBC_TASK_TYPE_CARTESIAN
 *          no_task_variables = 3
 *          root_frame = "Robot_base"
 *          tip_frame = "Elbow"
 *
 *        - task_type = WBC_TASK_TYPE_JOINT
 *          no_task_variables = total_no_of_robot_joints
 *
 *        - task_type = WBC_TASK_TYPE_JOINT
 *          no_task_variables = 3
 *          joints_ = ["J_1", "J_2", "J_3"]
 */
class SubTaskConfig{
public:
    /** Whole body task type, can be joint space or Cartesian for now */
    WBC_TASK_TYPE task_type_;

    /**
     * Only Cartesian Tasks: No of variables associated with this task.
     * E.g. a 6D Cartesian task would have 6 variables.
     * This parameter is neglected if the task is in joint space
     */
    uint no_task_variables_;

    /**
     * Only Cartesian Tasks: Root frame associated with this task.
     * Has to be the name of a link available in robot`s KDL tree.
     * This parameter is neglected if the task is in joint space
     */
    std::string root_frame_;

    /** Only Cartesian Tasks: Tip frame associated with this task.
     *  Has to be the name of a link available in robot`s KDL tree or an empty string.
     *  If empty, the task is assumed to in joint space
     */
    std::string tip_frame_;

    /** Only Joint Space Tasks: In case the task is of type WBC_TASK_TYPE_JOINT,
     * the joints used for this task can be specified.
     * If empty, all robot joints will be used */
    std::vector<std::string> joints_;
};

typedef std::vector< std::vector<Eigen::VectorXd> > WbcInput;
typedef std::vector< std::vector<SubTaskConfig> > WbcConfig;

/**
 * @brief helper class to carry sub task specific information
 */
class SubTask{
public:
    SubTask(const SubTaskConfig& config, const uint no_robot_joints){
        config_ = config;
        y_des_.resize(config.no_task_variables_);

        task_weights_.resize(config.no_task_variables_);
        task_weights_.setConstant(1);

        task_jac_ = KDL::Jacobian(config.no_task_variables_);
        task_jac_.data.setZero();

        H_.resize(config.no_task_variables_,6);
        H_.setZero();

        Uf_.resize(6, config.no_task_variables_);
        Uf_.setIdentity();

        Vf_.resize(config.no_task_variables_, config.no_task_variables_);
        Vf_.setIdentity();

        Sf_.resize(config.no_task_variables_);
        Sf_.setZero();

        A_.resize(config.no_task_variables_, no_robot_joints);
        A_.setZero();
    }

    ~SubTask(){}

    SubTaskConfig config_;

    Eigen::VectorXd y_des_;
    Eigen::VectorXd task_weights_;
    KDL::Jacobian task_jac_;
    KDL::Frame pose_;
    Eigen::MatrixXd A_;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf_, Vf_;
    Eigen::VectorXd Sf_;
    Eigen::MatrixXd H_;
};

#endif
