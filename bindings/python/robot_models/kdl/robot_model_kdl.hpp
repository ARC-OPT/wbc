#ifndef WBC_PY_ROBOT_MODEL_KDL_HPP
#define WBC_PY_ROBOT_MODEL_KDL_HPP

#include "robot_models/kdl/RobotModelKDL.hpp"
#include "../../wbc_types_conversions.h"

namespace wbc_py {

/** Wrapper for wbc::RobotModelKDL. Unfortunately we have to do this, since base::samples::Joints and other Rock types cannot be easily exposed to python*/
class RobotModelKDL : public wbc::RobotModelKDL{
public:
    bool configure(const wbc_py::RobotModelConfig &cfg);
    void update(const base::NamedVector<base::JointState> &joint_state);
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state);
    base::NamedVector<base::JointState> jointState2(const std::vector<std::string> &names);
    void setActiveContacts(const base::NamedVector<int> & active_contacts);
    base::NamedVector<int> getActiveContacts2();
    base::NamedVector<base::JointLimitRange> jointLimits2();
    wbc_py::RobotModelConfig getRobotModelConfig();
};

}

#endif

