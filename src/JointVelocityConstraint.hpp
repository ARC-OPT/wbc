#ifndef JOINTVELOCITYCONSTRAINT_HPP
#define JOINTVELOCITYCONSTRAINT_HPP

#include "JointConstraint.hpp"

namespace wbc{
/**
 * @brief Implementation of a Joint velocity constraint.
 */
class JointVelocityConstraint : public JointConstraint{
public:
    JointVelocityConstraint(ConstraintConfig config, uint n_robot_joints);
    virtual ~JointVelocityConstraint();

    /**
     * @brief Update the Joint reference input for this constraint.
     * @param ref Joint reference input. Vector size has ot be same number of constraint variables. Joint Names have to match the constraint joint names.
     * Each entry has to have a valid velocity. All other entries will be ignored.
     */
    virtual void setReference(const base::commands::Joints& ref);
};

} // namespace wbc

#endif
