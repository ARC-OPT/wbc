#ifndef WBC_TYPES_SPATIAL_ACCELERATION_HPP
#define WBC_TYPES_SPATIAL_ACCELERATION_HPP

#include "Pose.hpp"

namespace wbc { namespace types {

class SpatialAcceleration{
public:
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;

    void setZero(){
        linear.setZero();
        angular.setZero();
    }

    Eigen::VectorXd vector6d() const{
        Eigen::VectorXd v(6);
        v.segment(0,3) = linear;
        v.segment(3,3) = angular;
        return v;
    }
};

/** Transform of a spatial acceleration from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the adjoint \f$ Adj(X) \in R^{6 \times 6} \f$ of the given input transform \f$X = (R,p) \in SE(3)\f$ as follows:
 *  \f[
 *     \left(\begin{array}{cc} \dot{\omega} \\ \dot{v} \end{array}\right)_B = \left(\begin{array}{cc} R & 0 \\ \left[p\right]R & R \end{array}\right) \left(\begin{array}{cc} \dot{\omega} \\ \dot{v} \end{array}\right)_A
 *  \f]
 * with \n
 * \f[
 *     \left[p\right] = \left(\begin{array}{ccc}0 & -p_z & p_y \\ p_z & 0 &-p_x \\ -p_y & p_x & 0\end{array}\right)
 * \f]
 *  and \n
 *  \f$ \dot{\omega} \in R^3\f$ - Angular acceleration \n
 *  \f$ \dot{v} \in R^3\f$ - Linear acceleration \n
 *  \f$ R \in SO(3)\f$ - Rotation matrix \n
 *  \f$ p \in R^3\f$ - Translation vector \n\n
 *
 *  *According to: Lynch, K.M. and Park, F.C. 2017. Modern Robotics: Mechanics, Planning, and Control. page 100. Cambridge University Press, USA
 *  @param transform Input transform as position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param acc_in Input spatial acceleration, expressed in coordinate frame A
 *  @returns Spatial Acceleration in new coordinate frame B
 */
SpatialAcceleration operator*(const Pose& transform, const SpatialAcceleration& acc_in);

}
}

#endif
