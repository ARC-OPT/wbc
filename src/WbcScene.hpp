#ifndef WBCSCENE_HPP
#define WBCSCENE_HPP

#include "ConstraintConfig.hpp"
#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{

class RobotModel;
class Solver;

class WbcScene{
protected:
    std::vector< std::vector<Constraint*> > constraints;
    std::vector< ConstraintsPerPrio > constraint_vector;
    std::vector<int> n_constraint_variables_per_prio;
    RobotModel* robot_model;
    Solver* solver;

    /** Create a constraint and add it to the WBC scene*/
    virtual Constraint* createConstraint(const ConstraintConfig &config) = 0;

    /** Delete all constraints and free memory*/
    void clearConstraints();

public:
    WbcScene(RobotModel* robot_model, Solver* solver) :
        robot_model(robot_model),
        solver(solver){}

    virtual ~WbcScene(){}

    /** Configure the WBC scene. Create constraints and sort them by priority*/
    bool configure(const std::vector<ConstraintConfig> &config);

    /** Update the wbc scene with the (updated) robot model and return the current solver output*/
    virtual void solve(base::commands::Joints& ctrl_output) = 0;

    /** Return a Particular constraint. Throw if the constraint does not exist */
    Constraint* getConstraint(const std::string& name);

    /** True in case the given constraint exists */
    bool hasConstraint(const std::string& name);

    /** Returns all constraints as vector */
    std::vector< ConstraintsPerPrio > getConstraints();

    /** Sort constraint config by the priorities of the constraints */
    static void sortConstraintConfig(const std::vector<ConstraintConfig>& config, std::vector< std::vector<ConstraintConfig> >& sorted_config);

    /** Return number of constraints per priority, given the constraint config*/
    static std::vector<int> getNConstraintVariablesPerPrio(const std::vector<ConstraintConfig> &config);
};

} // namespace wbc

#endif
