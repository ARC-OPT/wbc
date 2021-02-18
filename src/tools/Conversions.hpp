#ifndef WBC_TYPES_CONVERSIONS_HPP
#define WBC_TYPES_CONVERSIONS_HPP

namespace base { namespace  samples {

class RigidBodyStateSE3;
class RigidBodyState;

void toRigidBodyState(const RigidBodyStateSE3& in, RigidBodyState& out);
void fromRigidBodyState(const RigidBodyState& in, RigidBodyStateSE3& out);

}
}

#endif
