#include "ControllerTools.hpp"

#include <base/Twist.hpp>
#include <base/Pose.hpp>

namespace base{

Twist operator-(const Pose& a, const Pose& b){
    Eigen::Affine3d tf_a = a.toTransform();
    Eigen::Affine3d tf_b = b.toTransform();

    Eigen::Matrix3d rot_mat = tf_b.rotation().inverse() * tf_a.rotation();
    Eigen::AngleAxisd angle_axis;
    angle_axis.fromRotationMatrix(rot_mat);

    base::Twist twist;
    twist.linear = tf_a.translation() - tf_b.translation();
    twist.angular = tf_b.rotation() * (angle_axis.axis() * angle_axis.angle());
    return twist;
}
}
