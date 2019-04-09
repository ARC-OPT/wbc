#ifndef WBC_TYPES_ACCELERATION_HPP
#define WBC_TYPES_ACCELERATION_HPP

#include <base/Eigen.hpp>

namespace wbc {

class Acceleration{
public:
    Acceleration();
    /** Set all members to NaN*/
    void setNaN();
    /** Set all members to zero/Identity*/
    void setZero();
    /** Return false if one of the entries is NaN*/
    bool isValid() const;

    /** Linear 3D acceleration in order x-y-z (m/ss)*/
    base::Vector3d linear;
    /** Angular 3D acceleration in order rot_x-rot_y-rot_z (rad/ss)*/
    base::Vector3d angular;
};

Acceleration operator+(const Acceleration& a, const Acceleration& b);
Acceleration operator*(const base::Vector6d& a, const Acceleration& b);

}

#endif
