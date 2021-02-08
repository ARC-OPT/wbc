#include "core/RobotModelConfig.hpp"
#include "robot_models/RobotModelHyrodyn.hpp"
#include "robot_models/RobotModelKDL.hpp"
#include <boost/python.hpp>

namespace bp = boost::python;

namespace wbc_py{

class Pose{
public:
    Pose(){
        for(int i = 0; i < 3; i++){
            position.append((double)base::NaN<double>());
            orientation.append((double)base::NaN<double>());
        }
        orientation.append((double)base::NaN<double>());
    }
    bp::list position;
    bp::list orientation;
};

class Twist{
public:
    Twist(){
        for(int i = 0; i < 3; i++){
            linear.append((double)base::NaN<double>());
            angular.append((double)base::NaN<double>());
        }
    }
    bp::list linear;
    bp::list angular;
};

class Acceleration{
public:
    Acceleration(){
        for(int i = 0; i < 3; i++){
            linear.append((double)base::NaN<double>());
            angular.append((double)base::NaN<double>());
        }
    }
    bp::list linear;
    bp::list angular;
};

class Wrench{
public:
    Wrench(){
        for(int i = 0; i < 3; i++){
            force.append((double)base::NaN<double>());
            torque.append((double)base::NaN<double>());
        }
    }
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
    RobotModelConfig() :
        world_frame_id("world"),
        floating_base(false){

    }
    std::string file;
    std::string submechanism_file;
    bp::list joint_names;
    bp::list actuated_joint_names;
    bool floating_base;
    std::string world_frame_id;
    RigidBodyStateSE3 floating_base_state;
    bp::list contact_points;
};

class RobotModel{
protected:
    wbc::RobotModelPtr robot_model;
public:
    bool configure(const RobotModelConfig& cfg);
    void update(bp::list names, bp::list positions, bp::list velocities, bp::list accelerations);
    void update2(bp::list names, bp::list positions, bp::list velocities, bp::list accelerations, RigidBodyStateSE3 floating_base_state);
    RigidBodyStateSE3 rigidBodyState(const std::string& root, const std::string &tip);
    bp::list spaceJacobian(const std::string& root, const std::string &tip);
    bp::list bodyJacobian(const std::string& root, const std::string &tip);
    Acceleration spatialAccelerationBias(const std::string& root, const std::string &tip);
    bp::list jointSpaceInertiaMatrix();
    bp::list biasForces();
    int noOfJoints(){return robot_model->noOfJoints();}
    bp::list jointNames();
    bp::list actuatedJointNames();
};

class RobotModelHyrodyn : public RobotModel{
public:
    RobotModelHyrodyn(){
        robot_model = std::make_shared<wbc::RobotModelHyrodyn>();
    }
};

class RobotModelKDL : public RobotModel{
public:
    RobotModelKDL(){
        robot_model = std::make_shared<wbc::RobotModelKDL>();
    }
};

}
