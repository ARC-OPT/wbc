#ifndef WBCSCENE_HPP
#define WBCSCENE_HPP

#include "ConstraintStatus.hpp"
#include "../types/QuadraticProgram.hpp"
#include <base/commands/Joints.hpp>
#include "RobotModel.hpp"
#include "QPSolver.hpp"

namespace wbc{

/**
 * @brief Base class for all wbc scenes.
 */
class WbcScene{
protected:
    RobotModelPtr robot_model;
    QPSolverPtr solver;
    std::vector< std::vector<ConstraintPtr> > constraints;
    ConstraintsStatus constraints_status;
    HierarchicalQP constraints_prio;
    std::vector<int> n_constraint_variables_per_prio;
    bool configured;
    base::commands::Joints solver_output_joints;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual ConstraintPtr createConstraint(const ConstraintConfig &config) = 0;

    /**
     * @brief Delete all constraints and free memory
     */
    void clearConstraints();

public:
    WbcScene(RobotModelPtr robot_model, QPSolverPtr solver) :
        robot_model(robot_model),
        solver(solver),
        configured(false){}

    virtual ~WbcScene(){
    }

    /**
     * @brief Configure the WBC scene. Create constraints and sort them by priority
     * @param Constraint configuration. Size has to be > 0. All constraints have to be valid. See ConstraintConfig.hpp for more details.
     */
    bool configure(const std::vector<ConstraintConfig> &config);

    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @return Hierarchical quadratic program (solver input)
     */
    virtual const HierarchicalQP& update() = 0;

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint command
     */
    virtual const base::commands::Joints& solve(const HierarchicalQP& hqp) = 0;

    /**
     * @brief Return a Particular constraint. Throw if the constraint does not exist
     */
    ConstraintPtr getConstraint(const std::string& name);

    /**
     * @brief True in case the given constraint exists
     */
    bool hasConstraint(const std::string& name);

    /**
     * @brief Returns all constraints as vector
     */
    const ConstraintsStatus& getConstraintsStatus(){return constraints_status;}

    /**
     * @brief Sort constraint config by the priorities of the constraints
     */
    static void sortConstraintConfig(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config);

    /**
     * @brief Return number of constraints per priority, given the constraint config
     */
    static std::vector<int> getNConstraintVariablesPerPrio(const std::vector<ConstraintConfig> &config);

    /**
     * @brief updateConstraintsStatus Evaluate the fulfillment of the constraints given the current robot state and the solver output
     */
    virtual const ConstraintsStatus &updateConstraintsStatus() = 0;

    /**
     * @brief Return constraints sorted by priority for the solver
     */
    void getHierarchicalQP(HierarchicalQP& hqp){hqp = constraints_prio;}

    /**
     * @brief Get current solver output
     */
    const base::commands::Joints& getSolverOutput(){return solver_output_joints;}
};

} // namespace wbc

#endif
