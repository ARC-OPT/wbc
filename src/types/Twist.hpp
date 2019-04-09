#ifndef WBC_TYPES_TWIST_HPP
#define WBC_TYPES_TWIST_HPP

#include <base/Eigen.hpp>

namespace wbc {

class Twist{
public:
    Twist();
    /** Set all members to NaN*/
    void setNaN();
    /** Set all members to zero/Identity*/
    void setZero();
    /** Return false if one of the entries is NaN*/
    bool isValid() const;

    /** Linear 3D velocity in order x-y-z (m/s)*/
    base::Vector3d linear;
    /** Angular 3D velocity in order rot_x-rot_y-rot_z (rad/s)*/
    base::Vector3d angular;

};

Twist operator+(const Twist& a, const Twist& b);
Twist operator*(const base::Vector6d& a, const Twist& b);

}

#endif
