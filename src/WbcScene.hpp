#ifndef WBCSCENE_HPP
#define WBCSCENE_HPP

#include "ConstraintConfig.hpp"
#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{

class RobotModel;
class Solver;

/**
 * @brief Base class for all wbc scenes
 */
class WbcScene{
protected:
    std::vector< std::vector<Constraint*> > constraints;
    std::vector< ConstraintsPerPrio > constraint_vector;
    std::vector<int> n_constraint_variables_per_prio;
    RobotModel* robot_model;
    Solver* solver;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual Constraint* createConstraint(const ConstraintConfig &config) = 0;

    /**
     * @brief Delete all constraints and free memory
     */
    void clearConstraints();

public:
    WbcScene(RobotModel* robot_model, Solver* solver) :
        robot_model(robot_model),
        solver(solver){}

    virtual ~WbcScene(){}

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
    Constraint* getConstraint(const std::string& name);

    /**
     * @brief True in case the given constraint exists
     */
    bool hasConstraint(const std::string& name);

    /**
     * @brief Returns all constraints as vector
     */
    std::vector< ConstraintsPerPrio > getConstraints();

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
