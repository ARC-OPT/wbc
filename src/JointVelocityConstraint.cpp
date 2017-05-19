#include "JointVelocityConstraint.hpp"

namespace wbc{

JointVelocityConstraint::JointVelocityConstraint(ConstraintConfig config, uint n_robot_joints)
    : Constraint(config, n_robot_joints){

}

void JointVelocityConstraint::setReference(const base::commands::Joints& ref){

    if(ref.size() != config.noOfConstraintVariables())
        throw std::invalid_argument("Constraint " + config.name + ": Size of reference input is "
                                    + std::to_string(ref.size()) + " but should be " + std::to_string(no_variables));

    // This value will also be used for checking the constraint timeout!
    this->time = ref.time;

    for(size_t i = 0; i < ref.size(); i++){
        uint idx;
        try{
            idx = ref.mapNameToIndex(config.joint_names[i]);
        }
        catch(std::exception e){
            throw std::invalid_argument("Constraint " + config.name + " expects joint "
                                        + config.joint_names[i] + " but this joint is not in reference vector");
        }

        if(!ref[idx].hasSpeed())
            throw std::invalid_argument("Constraint " + config.name + ": Reference input for joint " +
                                        config.joint_names[i] + " has no speed value");

        this->y_ref(i) = ref[idx].speed;
    }
}
} // namespace wbc
