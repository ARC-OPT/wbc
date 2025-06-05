#include "Wrench.hpp"

namespace wbc { namespace types {

Wrench operator*(const Pose& transform, const Wrench& wrench_in)
{
    Wrench wrench_out;
    wrench_out.force  = transform.orientation*wrench_in.force;
    wrench_out.torque = transform.orientation*wrench_in.torque - transform.position.cross(wrench_out.force);
    return wrench_out;
}

}
}
