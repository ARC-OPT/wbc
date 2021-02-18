#ifndef WBC_PY_ROBOT_MODELS_HPP
#define WBC_PY_ROBOT_MODELS_HPP

#include "robot_models/RobotModelKDL.hpp"
#include "robot_models/RobotModelHyrodyn.hpp"

namespace wbc_py {

/** Wrapper for wbc::RobotModelHyrodyn. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelHyrodyn : public wbc::RobotModelHyrodyn{
public:
    void update(const base::NamedVector<base::JointState> &joint_state);
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state);
};

/** Wrapper for wbc::RobotModelKDL. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelKDL : public wbc::RobotModelKDL{
public:
    void update(const base::NamedVector<base::JointState> &joint_state);
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state);
};

}

#endif

