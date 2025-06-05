#ifndef WBC_TYPES_WRENCH_HPP
#define WBC_TYPES_WRENCH_HPP

#include "Pose.hpp"

namespace wbc { namespace types {

class Wrench{
public:
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

    void setZero(){
        force.setZero();
        torque.setZero();
    }

    Eigen::VectorXd vector6d() const{
        Eigen::VectorXd v(6);
        v.segment(0,3) = force;
        v.segment(3,3) = torque;
        return v;
    }
};

/** Transform of a wrench \f$ F = (m,f)^T \f$ from a coordinate frame A to another coordinate frame B. The mapping is performed using
 *  the co-adjoint \f$ Adj(X)^{-T} \in R^{6 \times 6} \f$ of the given input transform \f$X = (R,p) \in SE(3)\f$ as follows*:
 *  \f[
 *     \left(\begin{array}{cc} m \\ f \end{array}\right)_B = \left(\begin{array}{cc} R & \left[p\right]R \\ 0 & R \end{array}\right) \left(\begin{array}{cc} m \\ f \end{array}\right)_A
 *  \f]
 * with \n
 * \f[
 *     \left[p\right] = \left(\begin{array}{ccc}0 & -p_z & p_y \\ p_z & 0 &-p_x \\ -p_y & p_x & 0\end{array}\right)
 * \f]
 *  and \n
 *  \f$ m \in R^3\f$ - Torque/moment \n
 *  \f$ f \in R^3\f$ - Linear force \n
 *  \f$ R \in SO(3)\f$ - Rotation matrix \n
 *  \f$ p \in R^3\f$ - Translation vector \n\n
 *
 *  *According to: Lynch, K.M. and Park, F.C. 2017. Modern Robotics: Mechanics, Planning, and Control. page 110. Cambridge University Press, USA
 *  @param transform Input transform as position and orientation of frame A expressed in frame B (not vice versa!)
 *  @param wrench_in Input twist, expressed in coordinate frame A
 *  @returns wrench in new coordinate frame B
 */
Wrench operator*(const Pose& transform, const Wrench& wrench_in);

}
}

#endif
