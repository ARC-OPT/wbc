#ifndef WBC_PY_ROBOT_MODEL_KDL_HPP
#define WBC_PY_ROBOT_MODEL_KDL_HPP

#include "robot_models/kdl/RobotModelKDL.hpp"

namespace wbc_py {

/** Wrapper for wbc::RobotModelKDL. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelKDL : public wbc::RobotModelKDL{
public:
    void update(const base::NamedVector<base::JointState> &joint_state);
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state);
    base::NamedVector<base::JointState> jointState2(const std::vector<std::string> &names);
};

}

#endif

