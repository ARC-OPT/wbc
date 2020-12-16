#ifndef CARTESIANACCELERATIONCONSTRAINT_HPP
#define CARTESIANACCELERATIONCONSTRAINT_HPP

#include "CartesianConstraint.hpp"
#include <base/Acceleration.hpp>

namespace wbc{

base::Vector6d operator+(base::Vector6d a, base::Acceleration b);
base::Vector6d operator-(base::Vector6d a, base::Acceleration b);

/**
 * @brief Implementation of a Cartesian acceleration constraint.
 */
class CartesianAccelerationConstraint : public CartesianConstraint{
public:
    CartesianAccelerationConstraint(ConstraintConfig config, uint n_robot_joints);
    virtual ~CartesianAccelerationConstraint();

    /**
     * @brief Update the Cartesian reference input for this constraint.
     * @param ref Reference input for this constraint. Only the acceleration part is relevant (Must have a valid linear and angular acceleration!)
     */
    virtual void setReference(const base::samples::RigidBodyStateSE3& ref);
};

} // namespace wbc

#endif
