#ifndef JOINTVELOCITYCONSTRAINT_HPP
#define JOINTVELOCITYCONSTRAINT_HPP

#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{

class JointVelocityConstraint : public Constraint{
public:
    JointVelocityConstraint(ConstraintConfig config, uint n_robot_joints);

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    void setReference(const base::commands::Joints& ref);
};

} // namespace wbc

#endif
