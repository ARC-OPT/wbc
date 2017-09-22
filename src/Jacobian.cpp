#include "Jacobian.hpp"

namespace wbc {

Jacobian::Jacobian(){

}

Jacobian::Jacobian(uint n_robot_joints){
    resize(6,n_robot_joints);
}

Jacobian::~Jacobian(){
}

void Jacobian::changeRefPoint(const base::Vector3d& v){
    base::Vector3d rot,vel; // helper variables
    for(int i = 0; i < cols(); i++){
        vel = col(i).segment(0,3);
        rot = col(i).segment(3,3);
        col(i).segment(0,3) = vel + rot.cross(v);
        col(i).segment(3,3) = rot;
    }

}
void Jacobian::changeRefFrame(const base::Affine3d& a){
    base::Vector3d rot,vel; // helper variables
    for(int i = 0; i < cols(); i++){
        vel = col(i).segment(0,3);
        rot = col(i).segment(3,3);
        col(i).segment(3,3) = a.rotation() * rot;
        rot = col(i).segment(3,3);
        col(i).segment(0,3) = a.rotation() * vel + a.translation().cross(rot);
    }
}

} // namespace wbc
