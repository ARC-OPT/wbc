#ifndef JOINTCONSTRAINT_HPP
#define JOINTCONSTRAINT_HPP

#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc {
/**
 * @brief Abstract interface for a constraint in joint space
 */
class JointConstraint : public Constraint{
public:
    JointConstraint(const ConstraintConfig& _config, uint n_robot_joints);
    virtual ~JointConstraint();

    /**
     * @brief Update the Joint reference input for this constraint.
     */
    virtual void setReference(const base::commands::Joints& ref) = 0;

};

} //namespace wbc

#endif
