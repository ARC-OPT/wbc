#ifndef WBC_PY_ROBOT_MODEL_HYRODYN_HPP
#define WBC_PY_ROBOT_MODEL_HYRODYN_HPP

#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"

namespace wbc_py {

/** Wrapper for wbc::RobotModelHyrodyn. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelHyrodyn : public wbc::RobotModelHyrodyn{
public:
    void update(const base::NamedVector<base::JointState> &joint_state);
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state);
};

}

#endif

