#ifndef JOINTCONSTRAINT_HPP
#define JOINTCONSTRAINT_HPP

#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc {

class JointConstraint : public Constraint{
public:
    JointConstraint(const ConstraintConfig& _config, uint n_robot_joints);
    virtual ~JointConstraint();

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::commands::Joints& ref) = 0;

};

} //namespace wbc

#endif
