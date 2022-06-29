#include "HardConstraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/Float.hpp>

namespace wbc{

HardConstraint::HardConstraint(){

}

HardConstraint::HardConstraint(Type type, uint n_robot_joints) :
{
    reset();
    A.resize(no_variables, n_robot_joints);
    A.setZero();
}

HardConstraint::HardConstraint(Type type, uint n_robot_joints, const base::VectorXi& mask)
{

    A.resize(no_variables, n_robot_joints);
    A.setZero();
}


HardConstraint::~HardConstraint(){

}

const Type HardConstraint::type() {
    return type;
} 

const base::MatrixXd& HardConstraint::A() {
    return A;
}

const base::VectorXd& HardConstraint::b() {
    return b;
}

const base::VectorXd& HardConstraint::lb() {
    return lb;
}

const base::VectorXd& HardConstraint::ub() {
    return ub;
}


void HardConstraint::setMask(const base::VectorXi& mask){
    if(config.nVariables() != mask.size()){
        LOG_ERROR("Constraint %s: Size of mask vector should be %i but is %i", config.name.c_str(), config.nVariables(), weights.size())
        throw std::invalid_argument("Invalid constraint weights");
    }

    for(uint i = 0; i < weights.size(); i++)
        if(weights(i) != 0 || weights(i) != 1){
            LOG_ERROR("Constraint %s: Mask values should be among {0 ,1}, but mask %i is %d", config.name.c_str(), i, weights(i));
            throw std::invalid_argument("Invalid constraint mask");
        }

    this->mask = mask;
}



}// namespace wbc
