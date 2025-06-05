#include "Twist.hpp"

namespace wbc { namespace types {

Twist operator*(const Pose& transform, const Twist& twist_in)
{
    Twist twist_out;
    twist_out.angular=transform.orientation*twist_in.angular;
    twist_out.linear=transform.orientation*twist_in.linear - transform.position.cross(twist_out.angular);
    return twist_out;
}

types::Twist operator-(const types::Pose& a, const types::Pose& b){
    Eigen::Affine3d tf_a,tf_b;
    tf_a = a.orientation;
    tf_a.pretranslate(a.position);
    tf_b = b.orientation;
    tf_b.pretranslate(b.position);

    Eigen::Matrix3d rot_mat = tf_b.rotation().inverse() * tf_a.rotation();
    Eigen::AngleAxisd angle_axis;
    angle_axis.fromRotationMatrix(rot_mat);

    types::Twist twist;
    twist.linear = tf_a.translation() - tf_b.translation();
    twist.angular = tf_b.rotation() * (angle_axis.axis() * angle_axis.angle());
    return twist;
}


}
}
