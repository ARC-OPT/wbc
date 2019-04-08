#ifndef CONSTRAINT_STATUS_HPP
#define CONSTRAINT_STATUS_HPP

#include "Constraint.hpp"

namespace wbc {

class ConstraintStatus{
public:
    /** Last time the constraint reference values was updated.*/
    base::Time time;

    /** Configuration of this constraint. See ConstraintConfig.hpp for more details.*/
    ConstraintConfig config;

    /** Constraint activation. Has to be between 0 and 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints.*/
    double activation;

    /** Can be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero.*/
    int timeout;

    /** Constraint weights in root coordinates.*/
    base::VectorXd weights;

    /** Reference input for this constraint in root coordinates. Can be either joint or a Cartesian space variables.*/
    base::VectorXd y_ref;

    /** Solution as computed by the solver for this constraint in root coordinates.*/
    base::VectorXd y_solution;

    /** Actual constraint as executed on the robot. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates.*/
    base::VectorXd y;
};

class ConstraintsStatus : public base::NamedVector<ConstraintStatus>{
};

}

#endif
