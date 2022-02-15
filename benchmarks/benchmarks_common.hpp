#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <map>
#include <base/JointLimits.hpp>

namespace wbc{

double stdDev(const base::VectorXd& vec);
double whiteNoise(const double std_dev);
base::samples::Joints randomJointState(const std::vector<std::string>& joint_names, const base::JointLimits& limits);
base::samples::RigidBodyStateSE3 randomFloatingBaseState(const base::RigidBodyStateSE3 &initial_state);
void toCSV(std::map<std::string, base::VectorXd> res, const std::string &filename);
}
