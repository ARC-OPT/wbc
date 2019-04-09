#ifndef WBC_TYPES_TOOLS_HPP
#define WBC_TYPES_TOOLS_HPP

namespace base { namespace samples { class RigidBodyState;} }

namespace wbc {

class CartesianState;

void toRigidBodyState(const CartesianState& in, base::samples::RigidBodyState& out);
void fromRigidBodyState(const base::samples::RigidBodyState& in, CartesianState& out);

}

#endif
