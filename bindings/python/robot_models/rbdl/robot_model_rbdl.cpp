#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "../../wbc_types_conversions.h"
#include "robot_model_rbdl.hpp"

namespace wbc_py {

bool RobotModelRBDL::configure(const wbc_py::RobotModelConfig &cfg){
    return wbc::RobotModelRBDL::configure(toRobotModelConfig(cfg));
}
void RobotModelRBDL::update(const base::NamedVector<base::JointState> &joint_state){
    wbc::RobotModelRBDL::update(tobaseSamplesJoints(joint_state));
}
void RobotModelRBDL::update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state){
    wbc::RobotModelRBDL::update(tobaseSamplesJoints(joint_state), floating_base_state);
}
base::NamedVector<base::JointState> RobotModelRBDL::jointState2(const std::vector<std::string> &names){
    return toNamedVector(wbc::RobotModelRBDL::jointState(names));
}
void RobotModelRBDL::setActiveContacts(const base::NamedVector<wbc::ActiveContact> & active_contacts){
    wbc::RobotModelRBDL::setActiveContacts(toActiveContacts(active_contacts));
}
base::NamedVector<wbc::ActiveContact> RobotModelRBDL::getActiveContacts2(){
    return fromActiveContacts(wbc::RobotModelRBDL::getActiveContacts());
}
base::NamedVector<base::JointLimitRange> RobotModelRBDL::jointLimits2(){
    return fromJointLimits(wbc::RobotModelRBDL::jointLimits());
}
wbc_py::RobotModelConfig RobotModelRBDL::getRobotModelConfig(){
    return fromRobotModelConfig(wbc::RobotModelRBDL::getRobotModelConfig());
}

}

BOOST_PYTHON_MODULE(robot_model_rbdl){

    np::initialize();

    py::class_<wbc_py::RobotModelRBDL>("RobotModelRBDL")
            .def("configure",               &wbc_py::RobotModelRBDL::configure)
            .def("update",                  &wbc_py::RobotModelRBDL::update)
            .def("update",                  &wbc_py::RobotModelRBDL::update2)
            .def("jointState",              &wbc_py::RobotModelRBDL::jointState2)
            .def("rigidBodyState",          &wbc_py::RobotModelRBDL::rigidBodyState, py::return_value_policy<py::copy_const_reference>())
            .def("spaceJacobian",           &wbc_py::RobotModelRBDL::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("bodyJacobian",            &wbc_py::RobotModelRBDL::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("spatialAccelerationBias", &wbc_py::RobotModelRBDL::spatialAccelerationBias, py::return_value_policy<py::copy_const_reference>())
            .def("jacobianDot",             &wbc_py::RobotModelRBDL::jacobianDot, py::return_value_policy<py::copy_const_reference>())
            .def("jointSpaceInertiaMatrix", &wbc_py::RobotModelRBDL::jointSpaceInertiaMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("biasForces",              &wbc_py::RobotModelRBDL::biasForces, py::return_value_policy<py::copy_const_reference>())
            .def("jointNames",              &wbc_py::RobotModelRBDL::jointNames, py::return_value_policy<py::copy_const_reference>())
            .def("actuatedJointNames",      &wbc_py::RobotModelRBDL::actuatedJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("independentJointNames",   &wbc_py::RobotModelRBDL::independentJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("jointIndex",              &wbc_py::RobotModelRBDL::jointIndex)
            .def("baseFrame",               &wbc_py::RobotModelRBDL::baseFrame, py::return_value_policy<py::copy_const_reference>())
            .def("jointLimits",             &wbc_py::RobotModelRBDL::jointLimits2)
            .def("selectionMatrix",         &wbc_py::RobotModelRBDL::selectionMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("hasLink",                 &wbc_py::RobotModelRBDL::hasLink)
            .def("hasJoint",                &wbc_py::RobotModelRBDL::hasJoint)
            .def("hasActuatedJoint",        &wbc_py::RobotModelRBDL::hasActuatedJoint)
            .def("centerOfMass",            &wbc_py::RobotModelRBDL::centerOfMass, py::return_value_policy<py::copy_const_reference>())
            .def("setActiveContacts",       &wbc_py::RobotModelRBDL::setActiveContacts)
            .def("getActiveContacts",       &wbc_py::RobotModelRBDL::getActiveContacts2)
            .def("noOfJoints",              &wbc_py::RobotModelRBDL::noOfJoints)
            .def("noOfActuatedJoints",      &wbc_py::RobotModelRBDL::noOfActuatedJoints)
            .def("setGravityVector",        &wbc_py::RobotModelRBDL::setGravityVector)
            .def("floatingBaseState",       &wbc_py::RobotModelRBDL::floatingBaseState, py::return_value_policy<py::copy_const_reference>())
            .def("getRobotModelConfig",     &wbc_py::RobotModelRBDL::getRobotModelConfig);
}


