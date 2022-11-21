#include "robot_model_pinocchio.hpp"
#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "../../wbc_types_conversions.h"

namespace wbc_py {

bool RobotModelPinocchio::configure(const wbc_py::RobotModelConfig &cfg){
    return wbc::RobotModelPinocchio::configure(toRobotModelConfig(cfg));
}
void RobotModelPinocchio::update(const base::NamedVector<base::JointState> &joint_state){
    wbc::RobotModelPinocchio::update(tobaseSamplesJoints(joint_state));
}
void RobotModelPinocchio::update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state){
    wbc::RobotModelPinocchio::update(tobaseSamplesJoints(joint_state), floating_base_state);
}
base::NamedVector<base::JointState> RobotModelPinocchio::jointState2(const std::vector<std::string> &names){
    return toNamedVector(wbc::RobotModelPinocchio::jointState(names));
}
void RobotModelPinocchio::setActiveContacts(const base::NamedVector<wbc::ActiveContact> & active_contacts){
    wbc::RobotModelPinocchio::setActiveContacts(toActiveContacts(active_contacts));
}
base::NamedVector<wbc::ActiveContact> RobotModelPinocchio::getActiveContacts2(){
    return fromActiveContacts(wbc::RobotModelPinocchio::getActiveContacts());
}
base::NamedVector<base::JointLimitRange> RobotModelPinocchio::jointLimits2(){
    return fromJointLimits(wbc::RobotModelPinocchio::jointLimits());
}
wbc_py::RobotModelConfig RobotModelPinocchio::getRobotModelConfig(){
    return fromRobotModelConfig(wbc::RobotModelPinocchio::getRobotModelConfig());
}

}

BOOST_PYTHON_MODULE(robot_model_pinocchio){

    np::initialize();

    py::class_<wbc_py::RobotModelPinocchio>("RobotModelPinocchio")
            .def("configure",               &wbc_py::RobotModelPinocchio::configure)
            .def("update",                  &wbc_py::RobotModelPinocchio::update)
            .def("update",                  &wbc_py::RobotModelPinocchio::update2)
            .def("jointState",              &wbc_py::RobotModelPinocchio::jointState2)
            .def("rigidBodyState",          &wbc_py::RobotModelPinocchio::rigidBodyState, py::return_value_policy<py::copy_const_reference>())
            .def("spaceJacobian",           &wbc_py::RobotModelPinocchio::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("bodyJacobian",            &wbc_py::RobotModelPinocchio::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("spatialAccelerationBias", &wbc_py::RobotModelPinocchio::spatialAccelerationBias, py::return_value_policy<py::copy_const_reference>())
            .def("jacobianDot",             &wbc_py::RobotModelPinocchio::jacobianDot, py::return_value_policy<py::copy_const_reference>())
            .def("jointSpaceInertiaMatrix", &wbc_py::RobotModelPinocchio::jointSpaceInertiaMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("biasForces",              &wbc_py::RobotModelPinocchio::biasForces, py::return_value_policy<py::copy_const_reference>())
            .def("jointNames",              &wbc_py::RobotModelPinocchio::jointNames, py::return_value_policy<py::copy_const_reference>())
            .def("actuatedJointNames",      &wbc_py::RobotModelPinocchio::actuatedJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("independentJointNames",   &wbc_py::RobotModelPinocchio::independentJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("jointIndex",              &wbc_py::RobotModelPinocchio::jointIndex)
            .def("baseFrame",               &wbc_py::RobotModelPinocchio::baseFrame, py::return_value_policy<py::copy_const_reference>())
            .def("jointLimits",             &wbc_py::RobotModelPinocchio::jointLimits2)
            .def("selectionMatrix",         &wbc_py::RobotModelPinocchio::selectionMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("hasLink",                 &wbc_py::RobotModelPinocchio::hasLink)
            .def("hasJoint",                &wbc_py::RobotModelPinocchio::hasJoint)
            .def("hasActuatedJoint",        &wbc_py::RobotModelPinocchio::hasActuatedJoint)
            .def("centerOfMass",            &wbc_py::RobotModelPinocchio::centerOfMass, py::return_value_policy<py::copy_const_reference>())
            .def("setActiveContacts",       &wbc_py::RobotModelPinocchio::setActiveContacts)
            .def("getActiveContacts",       &wbc_py::RobotModelPinocchio::getActiveContacts2)
            .def("noOfJoints",              &wbc_py::RobotModelPinocchio::noOfJoints)
            .def("noOfActuatedJoints",      &wbc_py::RobotModelPinocchio::noOfActuatedJoints)
            .def("setGravityVector",        &wbc_py::RobotModelPinocchio::setGravityVector)
            .def("floatingBaseState",       &wbc_py::RobotModelPinocchio::floatingBaseState, py::return_value_policy<py::copy_const_reference>())
            .def("getRobotModelConfig",     &wbc_py::RobotModelPinocchio::getRobotModelConfig);
}


