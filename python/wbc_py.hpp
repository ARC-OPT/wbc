#include "core/RobotModelConfig.hpp"
#include "robot_models/RobotModelHyrodyn.hpp"
#include <boost/python.hpp>

namespace bp = boost::python;

namespace wbc_py{

class Pose{
public:
    bp::list orientation;
    bp::list position;
};

class Twist{
public:
    bp::list linear;
    bp::list angular;
};

class Acceleration{
public:
    bp::list linear;
    bp::list angular;
};

class Wrench{
public:
    bp::list force;
    bp::list torque;
};

class RigidBodyStateSE3{
public:
    Pose pose;
    Twist twist;
    Acceleration acceleration;
    Wrench wrench;

    base::RigidBodyStateSE3 toRBS();
};

class RobotModelConfig{
public:
    std::string file;
    std::string submechanism_file;
    bp::list joint_names;
    bp::list actuated_joint_names;
    bool floating_base;
    std::string world_frame_id;
    RigidBodyStateSE3 floating_base_state;
    bp::list contact_points;
};

class RobotModelHyrodyn{
    wbc::RobotModelHyrodyn robot_model;
public:
    bool configure(const RobotModelConfig& cfg);
    void update(bp::list names, bp::list positions, bp::list velocities, bp::list accelerations, RigidBodyStateSE3 floating_base_state);
    RigidBodyStateSE3 rigidBodyState(const std::string& root, const std::string &tip);
    bp::list spaceJacobian(const std::string& root, const std::string &tip);
    bp::list bodyJacobian(const std::string& root, const std::string &tip);
    Acceleration spatialAccelerationBias(const std::string& root, const std::string &tip);
    bp::list jointSpaceInertiaMatrix();
    bp::list biasForces();
};
}
