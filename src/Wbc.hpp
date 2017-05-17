//#ifndef WBC_HPP
//#define WBC_HPP

//#include "ConstraintConfig.hpp"
//#include "Constraint.hpp"
//#include <base/samples/RigidBodyState.hpp>
//#include <base/commands/Joints.hpp>

//namespace wbc{

//class Constraint;
//class OptProblem;

///**
// * @brief The Wbc class represents the interface between the robot model and the solver. It retrieves the required information
// *        about the current state of kinematics and dynamics of the robot from the robot model and, based on this information,
// *        prepares the optimization problem for the solver.
// */
//class Wbc{
//protected:
//    std::vector< std::vector<Constraint*> > constraints;
//    std::vector< ConstraintsPerPrio > constraint_vector;

//public:
//    Wbc();
//    virtual ~Wbc();

//    /**
//     * @brief Interface method for configuring WBC. This should prepare all required data containers.
//     * @param config Constraint configuration for WBC. Size has to be > 0
//     * @param joint_names This defines the order of joints within all matrices and vectors (Jacobians, weight matrices, etc.). Has to contain all joints that
//     *                    are included in any of the configured constraints, that is the joint names in case of joint space constraints and
//     * @return true in case of success, false in case of failure
//     */
//    virtual bool configure(const std::vector<ConstraintConfig> &config,
//                           const std::vector<std::string> &joint_names) = 0;

//    /**
//     * @brief Given all required task frames, this will setup the equation system for the constraint solver
//     * @param task_frames A map of task
//     * @param equations
//     */
//    virtual void setupOptProblem(const std::vector<TaskFrame*> &task_frames, OptProblem &opt_problem) = 0;


//    /**
//     * @brief Return a Particular constraint. Throw if the constraint does not exist
//     */
//    Constraint* getConstraint(const std::string& name);

//    /**
//     * @brief hasConstraint True in case the given constraint exists
//     */
//    bool hasConstraint(const std::string& name);

//    /**
//     * @brief Return all constraints as vector
//     */
//    std::vector< ConstraintsPerPrio > getConstraints();

//    /**
//     * @brief Returns the number of constraint variables per priority
//     */
//    std::vector<int> getConstraintVariablesPerPrio();

//    /**
//     * @brief getNumberOfPriorities Returns number of priority levels
//     */
//    size_t getNumberOfPriorities(){return constraints.size();}

//    /**
//     * @brief sortConfigByPriority Sort WBC config by the priorities of the constraints
//     * @param config input config
//     * @param sorted_config output config
//     */
//    static void sortConfigByPriority(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config);
//};
//}

//#endif
