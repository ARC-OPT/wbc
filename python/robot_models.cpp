#include "eigen_conversion.h"
#include "base_types_conversion.h"
#include "std_vector_conversion.h"
#include "robot_models/RobotModelKDL.hpp"
#include "robot_models/RobotModelHyrodyn.hpp"

namespace wbc_py {

base::samples::Joints tobaseSamplesJoints(const base::NamedVector<base::JointState> &joint_state){
    base::samples::Joints joints;
    joints.elements = joint_state.elements;
    joints.names = joint_state.names;
    joints.time = base::Time::now();
    return joints;
}

/** Wrapper for wbc::RobotModelHyrodyn. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelHyrodyn : public wbc::RobotModelHyrodyn{
public:
    void update(const base::NamedVector<base::JointState> &joint_state){
        wbc::RobotModelHyrodyn::update(tobaseSamplesJoints(joint_state));
    }
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state){
        wbc::RobotModelHyrodyn::update(tobaseSamplesJoints(joint_state), floating_base_state);
    }
};

/** Wrapper for wbc::RobotModelKDL. Unfortunately we have to do this, since base::samples::Joints cannot be easily exposed to python*/
class RobotModelKDL : public wbc::RobotModelKDL{
public:
    void update(const base::NamedVector<base::JointState> &joint_state){
        wbc::RobotModelKDL::update(tobaseSamplesJoints(joint_state));
    }
    void update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state){
        wbc::RobotModelKDL::update(tobaseSamplesJoints(joint_state), floating_base_state);
    }
};

}

BOOST_PYTHON_MODULE(robot_models){

    np::initialize();

    py::class_<wbc_py::RobotModelHyrodyn>("RobotModelHyrodyn")
            .def("configure",               &wbc_py::RobotModelHyrodyn::configure)
            .def("update",                  &wbc_py::RobotModelHyrodyn::update)
            .def("update",                  &wbc_py::RobotModelHyrodyn::update2)
            .def("rigidBodyState",          &wbc_py::RobotModelHyrodyn::rigidBodyState, py::return_value_policy<py::copy_const_reference>())
            .def("spaceJacobian",           &wbc_py::RobotModelHyrodyn::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("bodyJacobian",            &wbc_py::RobotModelHyrodyn::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("jacobianDot",             &wbc_py::RobotModelHyrodyn::jacobianDot, py::return_value_policy<py::copy_const_reference>())
            .def("spatialAccelerationBias", &wbc_py::RobotModelHyrodyn::spatialAccelerationBias, py::return_value_policy<py::copy_const_reference>())
            .def("jointSpaceInertiaMatrix", &wbc_py::RobotModelHyrodyn::jointSpaceInertiaMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("biasForces",              &wbc_py::RobotModelHyrodyn::biasForces, py::return_value_policy<py::copy_const_reference>())
            .def("noOfJoints",              &wbc_py::RobotModelHyrodyn::noOfJoints)
            .def("noOfActuatedJoints",      &wbc_py::RobotModelHyrodyn::noOfActuatedJoints)
            .def("jointNames",              &wbc_py::RobotModelHyrodyn::jointNames, py::return_value_policy<py::copy_const_reference>())
            .def("actuatedJointNames",      &wbc_py::RobotModelHyrodyn::actuatedJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("setGravityVector",        &wbc_py::RobotModelHyrodyn::setGravityVector)
            .def("getGravityVector",        &wbc_py::RobotModelHyrodyn::getGravityVector, py::return_value_policy<py::copy_const_reference>())
            .def("selectionMatrix",         &wbc_py::RobotModelHyrodyn::selectionMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("floatingBaseState",       &wbc_py::RobotModelHyrodyn::floatingBaseState, py::return_value_policy<py::copy_const_reference>())
            .def("setActiveContacts",       &wbc_py::RobotModelHyrodyn::setActiveContacts)
            .def("getActiveContacts",       &wbc_py::RobotModelHyrodyn::getActiveContacts, py::return_value_policy<py::copy_const_reference>())
            .def("setContactPoints",        &wbc_py::RobotModelHyrodyn::setContactPoints)
            .def("getContactPoints",        &wbc_py::RobotModelHyrodyn::getContactPoints, py::return_value_policy<py::copy_const_reference>())
            .def("hasLink",                 &wbc_py::RobotModelHyrodyn::hasLink)
            .def("hasJoint",                &wbc_py::RobotModelHyrodyn::hasJoint);

    py::class_<wbc_py::RobotModelKDL>("RobotModelKDL")
            .def("configure",               &wbc_py::RobotModelKDL::configure)
            .def("update",                  &wbc_py::RobotModelKDL::update)
            .def("update",                  &wbc_py::RobotModelKDL::update2)
            .def("rigidBodyState",          &wbc_py::RobotModelKDL::rigidBodyState, py::return_value_policy<py::copy_const_reference>())
            .def("spaceJacobian",           &wbc_py::RobotModelKDL::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("bodyJacobian",            &wbc_py::RobotModelKDL::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("jacobianDot",             &wbc_py::RobotModelKDL::jacobianDot, py::return_value_policy<py::copy_const_reference>())
            .def("spatialAccelerationBias", &wbc_py::RobotModelKDL::spatialAccelerationBias, py::return_value_policy<py::copy_const_reference>())
            .def("jointSpaceInertiaMatrix", &wbc_py::RobotModelKDL::jointSpaceInertiaMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("biasForces",              &wbc_py::RobotModelKDL::biasForces, py::return_value_policy<py::copy_const_reference>())
            .def("noOfJoints",              &wbc_py::RobotModelKDL::noOfJoints)
            .def("noOfActuatedJoints",      &wbc_py::RobotModelKDL::noOfActuatedJoints)
            .def("jointNames",              &wbc_py::RobotModelKDL::jointNames, py::return_value_policy<py::copy_const_reference>())
            .def("actuatedJointNames",      &wbc_py::RobotModelKDL::actuatedJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("setGravityVector",        &wbc_py::RobotModelKDL::setGravityVector)
            .def("getGravityVector",        &wbc_py::RobotModelKDL::getGravityVector, py::return_value_policy<py::copy_const_reference>())
            .def("selectionMatrix",         &wbc_py::RobotModelKDL::selectionMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("floatingBaseState",       &wbc_py::RobotModelKDL::floatingBaseState, py::return_value_policy<py::copy_const_reference>())
            .def("setActiveContacts",       &wbc_py::RobotModelKDL::setActiveContacts)
            .def("getActiveContacts",       &wbc_py::RobotModelKDL::getActiveContacts, py::return_value_policy<py::copy_const_reference>())
            .def("setContactPoints",        &wbc_py::RobotModelKDL::setContactPoints)
            .def("getContactPoints",        &wbc_py::RobotModelKDL::getContactPoints, py::return_value_policy<py::copy_const_reference>())
            .def("hasLink",                 &wbc_py::RobotModelKDL::hasLink)
            .def("hasJoint",                &wbc_py::RobotModelKDL::hasJoint);
}


