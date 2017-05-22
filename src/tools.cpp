#include "tools.hpp"

namespace wbc{

void pose_diff(const base::samples::RigidBodyState& a, const base::samples::RigidBodyState& b, const double dt, base::Vector6d& twist){
    pose_diff(a.getTransform(), b.getTransform(), dt, twist);
}

void pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b, const double dt, base::Vector6d& twist){

    Eigen::Matrix3d rot_mat = a.rotation().inverse() * b.rotation();
    Eigen::AngleAxisd angle_axis;
    angle_axis.fromRotationMatrix(rot_mat);

    twist.segment(0,3) = (b.translation() - a.translation())/dt;
    twist.segment(3,3) = a.rotation() * (angle_axis.axis() * angle_axis.angle())/dt;
}
} // namespace wbc
