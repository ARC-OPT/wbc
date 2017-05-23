#ifndef CARTESIANVELOCITYCONSTRAINT_HPP
#define CARTESIANVELOCITYCONSTRAINT_HPP

#include "CartesianConstraint.hpp"
#include "Jacobian.hpp"

namespace wbc{

/**
 * @brief Implementation of a Cartesian velocity constraint.
 */
class CartesianVelocityConstraint : public CartesianConstraint{
public:
    CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints);

    /**
     * @brief Update the Cartesian reference input for this constraint.
     * @param ref Reference input for this constraint. Only the velocity part is relevant (Must have a valid velocity and angular_velocity!)
     */
    virtual void setReference(const base::samples::RigidBodyState& ref);

    //Helper variables required for svd
    Jacobian jacobian;
    base::MatrixXd Uf, Vf;
    base::VectorXd Sf;
    base::MatrixXd H;
    base::VectorXd tmp;
};

} // namespace wbc

#endif
