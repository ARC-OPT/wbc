#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "ConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/Float.hpp>

namespace wbc{

/**
 * @brief Class to carry constraint specific information
 */
class Constraint{
public:

    Constraint();

    /** Resize all members accordingly*/
    Constraint(const ConstraintConfig& _config, uint n_robot_joints);
    ~Constraint();

    void reset();

    /** Check if the constraint is in timeout and set the timeout flag accordingly. A constraint is in timeout if
     *    - No reference value has been set yet (call of setReference())
     *    - A timeout value is configured (config.timeout > 0) and no reference value arrived during the timeout period
     */
    void checkTimeout();

    /** Set constraint weights. Size of weight vector has to be same as number of constraint variables and all weights have to be >= 0 */
    void setWeights(const base::VectorXd& weights);

    /** Set constraint activation. Value has to be between 0 and 1. Can be used to activate(1)/deactivate(0) the constraint.*/
    void setActivation(const double activation);

    /** Last time an update happened on this constraint*/
    base::Time time;

    /** Configuration of this constraint. See ConstraintConfig.hpp for more details*/
    ConstraintConfig config;

    /** Reference input for this constraint. Either joint or a Cartesian space variables in ref_frame coordinates*/
    base::VectorXd y_ref;

    /** Reference value for this constraint. Either joint or a Cartesian space variables transformed to root coordinates*/
    base::VectorXd y_ref_root;

    /** Constraint weights, a 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution*/
    base::VectorXd weights;

    /** Constraint weights, converted to root coordinates */
    base::VectorXd weights_root;

    /** Between 0 .. 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints */
    double activation;

    /** Can be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero*/
    int timeout;

    /** Number of constraint variables */
    uint no_variables;

    /** Solution as computed by the solver for this constraint. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y_solution;

    /** Actual constraint as executed on the robot. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y;

    /** Constraint matrix */
    base::MatrixXd A;
};
typedef std::vector<Constraint> ConstraintsPerPrio;

} // namespace wbc
#endif
