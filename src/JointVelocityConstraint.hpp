#ifndef JOINTVELOCITYCONSTRAINT_HPP
#define JOINTVELOCITYCONSTRAINT_HPP

#include "Constraint.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{

class JointVelocityConstraint : public Constraint{
public:
    JointVelocityConstraint(ConstraintConfig config) : Constraint(config){}

    /** Update the Cartesian reference input for this constraint. If the Constraint is a joint space
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::samples::RigidBodyState& ref){
        throw std::invalid_argument("Constraint " + config.name + " is a Cartesian space constraint, but you are trying to update it with a joint space reference!");
    }

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::commands::Joints& reference){

        if(ref.size() != no_variables)
            throw std::invalid_argument("Constraint " + name + ": Size of reference input is "
                                        + std::to_string(ref.size()) + " but should be " + std::to_string(no_variables));

        // This value will also be used for checking the constraint timeout!
        this->time = ref.time;

        for(size_t i = 0; i < reference.size(); i++){
            uint idx;
            try{
                idx = reference.mapNameToIndex(constraint->config.joint_names[i]);
            }
            catch(std::exception e){
                throw std::invalid_argument("Constraint " + name + " expects joint "
                                            + config.joint_names + " but this joint is not in reference vector");
            }

            if(!ref[idx].hasSpeed())
                throw std::invalid_argument("Constraint " + name + ": Reference input for joint " +
                                             config.joint_names[i] + " has no speed value");

            this->y_ref(i) = reference[idx].speed;
        }
    }

};

} // namespace wbc

#endif
