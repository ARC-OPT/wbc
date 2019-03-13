#ifndef JOINTACCELERATIONCONSTRAINT_HPP
#define JOINTACCELERATIONCONSTRAINT_HPP

#include "JointConstraint.hpp"

namespace wbc{
/**
 * @brief Implementation of a Joint velocity constraint.
 */
class JointAccelerationConstraint : public JointConstraint{
public:
    JointAccelerationConstraint(ConstraintConfig config, uint n_robot_joints);
    virtual ~JointAccelerationConstraint();

    /**
     * @brief Update the Joint reference input for this constraint.
     * @param ref Joint reference input. Vector size has ot be same number of constraint variables. Joint Names have to match the constraint joint names.
     * Each entry has to have a valid acceleration. All other entries will be ignored.
     */
    virtual void setReference(const base::commands::Joints& ref);
};

} // namespace wbc

#endif
