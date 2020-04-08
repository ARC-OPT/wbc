#include "CartesianConstraint.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc {

CartesianConstraint::CartesianConstraint(const ConstraintConfig &_config, uint n_robot_joints) :
    Constraint(_config, n_robot_joints){

}

CartesianConstraint::~CartesianConstraint(){

}

void CartesianConstraint::setReference(const base::samples::RigidBodyStateSE3 &ref){
    // TODO: Changing the reference frame at runtime may lead to a lot of confusion. Better remove this feature?
    if(!ref.frame_id.empty())
        this->config.ref_frame = ref.frame_id;
}

} //namespace wbc
