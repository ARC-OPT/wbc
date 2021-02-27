#ifndef CARTESIANVELOCITYCONSTRAINT_HPP
#define CARTESIANVELOCITYCONSTRAINT_HPP

#include "CartesianConstraint.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian velocity constraint.
 */
class CartesianVelocityConstraint : public CartesianConstraint{
public:
    CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints);
    virtual ~CartesianVelocityConstraint();

    /**
     * @brief Update the Cartesian reference input for this constraint.
     * @param ref Reference input for this constraint. Only the velocity part is relevant (Must have a valid linear and angular velocity!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref);
};

typedef std::shared_ptr<CartesianVelocityConstraint> CartesianVelocityConstraintPtr;

} // namespace wbc

#endif
