#ifndef CARTESIANVELOCITYCONSTRAINT_HPP
#define CARTESIANVELOCITYCONSTRAINT_HPP

#include "CartesianConstraint.hpp"

namespace wbc{

class CartesianVelocityConstraint : public CartesianConstraint{
public:
    CartesianVelocityConstraint(ConstraintConfig config) : CartesianConstraint(config){}

    /** Update the Cartesian reference input for this constraint. If the Constraint is a joint space
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::samples::RigidBodyState& ref){

        if(!ref.hasValidVelocity() ||
           !ref.hasValidAngularVelocity())
            throw std::invalid_argument("Constraint " + name + " has invalid velocity and/or angular velocity");

        this->time = reference.time;
        this->y_ref.segment(0,3) = ref.velocity;
        this->y_ref.segment(3,3) = this.angular_velocity;
    }

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::commands::Joints& ref){
        throw std::invalid_argument("Constraint " + config.name + " is a joint space constraint, but you are trying to update it with a Cartesian reference!");
    }
};

} // namespace wbc

#endif
