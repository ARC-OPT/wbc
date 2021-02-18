#include "JointAccelerationConstraint.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

JointAccelerationConstraint::JointAccelerationConstraint(ConstraintConfig config, uint n_robot_joints)
    : JointConstraint(config, n_robot_joints){

}

JointAccelerationConstraint::~JointAccelerationConstraint(){

}

void JointAccelerationConstraint::setReference(const base::commands::Joints& ref){

    if(ref.size() != config.nVariables()){
        LOG_ERROR("Constraint %s: Size of reference input is %i, but should be %i", config.name.c_str(), ref.size(), config.nVariables());
        throw std::invalid_argument("Invalid constraint reference input");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;

    for(size_t i = 0; i < ref.size(); i++){
        uint idx;
        try{
            idx = ref.mapNameToIndex(config.joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Constraint %s expects joint %s  but this joint is not in reference vector", config.name.c_str(), config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid constraint reference input");
        }

        if(!ref[idx].hasAcceleration()){
            LOG_ERROR("Constraint %s: Reference input for joint for joint %s has no valid acceleration value", config.name.c_str(), config.joint_names[i].c_str());
            throw std::invalid_argument("Invalid constraint reference input");
        }

        this->y_ref(i) = ref[idx].acceleration;
    }
}
} // namespace wbc
