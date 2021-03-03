#ifndef WBC_PY_SCENES_HPP
#define WBC_PY_SCENES_HPP

#include "scenes/VelocityScene.hpp"
#include "scenes/AccelerationSceneTSID.hpp"
#include "robot_models.hpp"
#include "solvers.hpp"

namespace wbc_py {

wbc::JointWeights toJointWeights(const base::NamedVector<double> &weights);

class VelocityScene : public wbc::VelocityScene{
public:
    VelocityScene(std::shared_ptr<RobotModelKDL> robot_model, std::shared_ptr<HierarchicalLSSolver> solver);
    VelocityScene(std::shared_ptr<RobotModelKDL> robot_model, std::shared_ptr<QPOASESSolver> solver);
    void setJointReference(const std::string& constraint_name, const base::NamedVector<base::JointState>& ref);
    void setCartReference(const std::string& constraint_name, const base::samples::RigidBodyStateSE3& ref);
    void setJointWeights(const base::NamedVector<double> &weights);
    base::NamedVector<double> getJointWeights2();
    base::NamedVector<double> getActuatedJointWeights2();
    base::NamedVector<base::JointState> solve2(const wbc::HierarchicalQP &hqp);
    base::NamedVector<wbc::ConstraintStatus> updateConstraintsStatus2();
};

class AccelerationSceneTSID : public wbc::AccelerationSceneTSID{
public:
    AccelerationSceneTSID(std::shared_ptr<RobotModelHyrodyn> robot_model, std::shared_ptr<QPOASESSolver> solver);
    void setJointReference(const std::string& constraint_name, const base::NamedVector<base::JointState>& ref);
    void setCartReference(const std::string& constraint_name, const base::samples::RigidBodyStateSE3& ref);
    void setJointWeights(const base::NamedVector<double> &weights);
    base::NamedVector<double> getJointWeights2();
    base::NamedVector<double> getActuatedJointWeights2();
    base::NamedVector<base::JointState> solve2(const wbc::HierarchicalQP &hqp);
    base::NamedVector<wbc::ConstraintStatus> updateConstraintsStatus2();
    base::NamedVector<base::Wrench> getContactWrenches();
};


}

#endif
