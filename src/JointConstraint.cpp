#include "JointConstraint.hpp"

namespace wbc {

JointConstraint::JointConstraint(const ConstraintConfig& _config, uint n_robot_joints) :
    Constraint(_config, n_robot_joints){

}

JointConstraint::~JointConstraint(){

}

} //namespace wbc
