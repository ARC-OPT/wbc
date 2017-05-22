#ifndef JOINTVELOCITYCONSTRAINT_HPP
#define JOINTVELOCITYCONSTRAINT_HPP

#include "JointConstraint.hpp"

namespace wbc{

class JointVelocityConstraint : public JointConstraint{
public:
    JointVelocityConstraint(ConstraintConfig config, uint n_robot_joints);
    virtual ~JointVelocityConstraint();

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::commands::Joints& ref);
};

} // namespace wbc

#endif
