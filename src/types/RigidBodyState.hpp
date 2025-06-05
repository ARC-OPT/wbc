#ifndef WBC_TYPES_RIGIDBODYSTATE_HPP
#define WBC_TYPES_RIGIDBODYSTATE_HPP

#include "Pose.hpp"
#include "Twist.hpp"
#include "SpatialAcceleration.hpp"

namespace wbc { namespace types {

class RigidBodyState{
public:
    Pose pose;
    Twist twist;
    SpatialAcceleration acceleration;
};

} // namespace types
} // namespace wbc

#endif // WBC_TYPES_RIGIDBODYSTATE_HPP
