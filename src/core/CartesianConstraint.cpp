#include "CartesianConstraint.hpp"
#include <ctrl_types/CartesianState.hpp>

namespace wbc {

CartesianConstraint::CartesianConstraint(const ConstraintConfig &_config, uint n_robot_joints) :
    Constraint(_config, n_robot_joints){

}

CartesianConstraint::~CartesianConstraint(){

}

void CartesianConstraint::setReference(const base::samples::CartesianState &ref){
    this->config.ref_frame = ref.target_frame;
}

} //namespace wbc
