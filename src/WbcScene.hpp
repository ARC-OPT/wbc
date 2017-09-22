#ifndef WBCSCENE_HPP
#define WBCSCENE_HPP

#include "ConstraintConfig.hpp"
#include "Constraint.hpp"
#include <base/commands/Joints.hpp>
#include <memory>

namespace wbc{

class RobotModel;
class Solver;

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<Solver> SolverPtr;

/**
 * @brief Base class for all wbc scenes
 */
class WbcScene{
protected:
    RobotModelPtr robot_model;
    SolverPtr solver;
    std::vector< std::vector<ConstraintPtr> > constraints;
    std::vector<int> n_constraint_variables_per_prio;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual ConstraintPtr createConstraint(const ConstraintConfig &config) = 0;

    /**
     * @brief Delete all constraints and free memory
     */
    void clearConstraints();

public:
    WbcScene(RobotModelPtr robot_model, SolverPtr solver) :
        robot_model(robot_model),
        solver(solver){}

    virtual ~WbcScene(){
    }

    /**
     * @brief Configure the WBC scene. Create constraints and sort them by priority
     * @param Constraint configuration. Size has to be > 0. All constraints have to be valid. See ConstraintConfig.hpp for more details.
     */
    bool configure(const std::vector<ConstraintConfig> &config);

    /**
     * @brief Update the wbc scene with the (updated) robot model and return the current solver output
     * @param ctrl_output Control solution that fulfill the given constraints as good as possible
     */
    virtual void solve(base::commands::Joints& ctrl_output) = 0;

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
    void getConstraints(std::vector<ConstraintsPerPrio>& constr_vect);

    /**
     * @brief Sort constraint config by the priorities of the constraints
     */
    static void sortConstraintConfig(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config);

    /**
     * @brief Return number of constraints per priority, given the constraint config
     */
    static std::vector<int> getNConstraintVariablesPerPrio(const std::vector<ConstraintConfig> &config);
};

} // namespace wbc

#endif
