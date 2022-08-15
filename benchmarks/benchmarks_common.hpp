#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <map>
#include <base/JointLimits.hpp>
#include <core/Scene.hpp>

namespace wbc{

double stdDev(const base::VectorXd& vec);
double whiteNoise(const double std_dev);
base::samples::Joints randomJointState(base::samples::Joints joint_state_in);
base::samples::Joints randomJointState(base::JointLimits limits);
base::samples::RigidBodyStateSE3 randomFloatingBaseState(base::samples::RigidBodyStateSE3 floating_base_state_in);
base::samples::RigidBodyStateSE3 randomFloatingBaseState();
void toCSV(std::map<std::string, base::VectorXd> res, const std::string &filename);
base::samples::RigidBodyStateSE3 randomConstraintReference();
std::map<std::string, base::VectorXd> evaluateWBCSceneRandom(WbcScenePtr scene, int n_samples);

}
