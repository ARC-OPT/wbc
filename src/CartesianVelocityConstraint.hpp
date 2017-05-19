#ifndef CARTESIANVELOCITYCONSTRAINT_HPP
#define CARTESIANVELOCITYCONSTRAINT_HPP

#include "Constraint.hpp"
#include "Jacobian.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class CartesianVelocityConstraint : public Constraint{
public:
    CartesianVelocityConstraint(ConstraintConfig config, uint n_robot_joints);

    /** Update the Cartesian reference input for this constraint. If the Constraint is a joint space
     *  constraint, you should throw an exception*/
    void setReference(const base::samples::RigidBodyState& ref);

    //Helper variables required for svd
    Jacobian jacobian;
    base::MatrixXd Uf, Vf;
    base::VectorXd Sf;
    base::MatrixXd H;
    base::VectorXd tmp;
};

} // namespace wbc

#endif
