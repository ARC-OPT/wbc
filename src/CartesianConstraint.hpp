#ifndef CARTESIANCONSTRAINT_HPP
#define CARTESIANCONSTRAINT_HPP

#include "Constraint.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace wbc {

class CartesianConstraint : public Constraint{
public:
    CartesianConstraint(const ConstraintConfig& _config, uint n_robot_joints);
    virtual ~CartesianConstraint();

    /** Update the Cartesian reference input for this constraint. If the Constraint is a joint space
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::samples::RigidBodyState& ref) = 0;
};

} //namespace wbc

#endif
