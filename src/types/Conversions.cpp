#include "Conversions.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace base { namespace samples {

void toRigidBodyState(const RigidBodyStateSE3& in, RigidBodyState& out){
    out.time = in.time;
    out.targetFrame = in.frame_id;
    out.setPose(in.pose);
    out.velocity         = in.twist.linear;
    out.angular_velocity = in.twist.angular;
}

void fromRigidBodyState(const RigidBodyState& in, RigidBodyStateSE3& out){
    out.time = in.time;
    out.frame_id = in.targetFrame;
    out.pose = in.getPose();
    out.twist.linear = in.velocity;
    out.twist.angular = in.angular_velocity;
}

}
}
