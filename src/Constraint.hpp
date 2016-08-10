#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include <base/Eigen.hpp>
#include "ConstraintConfig.hpp"
#include <base/time.h>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>
#include <base/logging.h>

namespace wbc{

/**
 * @brief Class to carry constraint specific information
 */
class Constraint{
public:

    Constraint();
    Constraint(const ConstraintConfig& _config);

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
     *  config.timeout time, this values will be set to zero*/
    int constraint_timed_out;

    /** Number of constraint variables */
    uint no_variables;

    /** Solution as computed by the solver for this constraint. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y_solution;

    /** Actual constraint as executed on the robot. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y;

    void reset();
};
typedef std::vector<Constraint> ConstraintsPerPrio;

}
#endif
