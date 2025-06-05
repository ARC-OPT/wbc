#include "SpatialAcceleration.hpp"

namespace wbc { namespace types {

SpatialAcceleration operator*(const Pose& transform, const SpatialAcceleration& acc_in)
{
    SpatialAcceleration acc_out;
    acc_out.angular = transform.orientation*acc_in.angular;
    acc_out.linear  = transform.orientation*acc_in.linear - transform.position.cross(acc_out.angular);
    return acc_out;
}

}
}
