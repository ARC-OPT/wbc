#include "tools.hpp"
#include "CartesianState.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace wbc {

void toRigidBodyState(const CartesianState& in, base::samples::RigidBodyState& out){
    out.time = in.time;
    out.setPose(in.pose);
    out.velocity         = in.twist.linear;
    out.angular_velocity = in.twist.angular;
}

void fromRigidBodyState(const base::samples::RigidBodyState& in, CartesianState& out){
    out.time = in.time;
    out.pose = in.getPose();
    out.twist.linear = in.velocity;
    out.twist.angular = in.angular_velocity;
}

}
