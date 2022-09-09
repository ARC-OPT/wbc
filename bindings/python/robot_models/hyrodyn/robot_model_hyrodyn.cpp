#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "robot_model_hyrodyn.hpp"

namespace wbc_py {

bool RobotModelHyrodyn::configure(const wbc_py::RobotModelConfig &cfg){
    return wbc::RobotModelHyrodyn::configure(toRobotModelConfig(cfg));
}
void RobotModelHyrodyn::update(const base::NamedVector<base::JointState> &joint_state){
    wbc::RobotModelHyrodyn::update(tobaseSamplesJoints(joint_state));
}
void RobotModelHyrodyn::update2(const base::NamedVector<base::JointState> &joint_state, const base::samples::RigidBodyStateSE3 &floating_base_state){
    wbc::RobotModelHyrodyn::update(tobaseSamplesJoints(joint_state), floating_base_state);
}
base::NamedVector<base::JointState> RobotModelHyrodyn::jointState2(const std::vector<std::string> &names){
    return toNamedVector(wbc::RobotModelHyrodyn::jointState(names));
}
void RobotModelHyrodyn::setActiveContacts(const base::NamedVector<int> & active_contacts){
    wbc::RobotModelHyrodyn::setActiveContacts(toActiveContacts(active_contacts));
}
base::NamedVector<int> RobotModelHyrodyn::getActiveContacts2(){
    return fromActiveContacts(wbc::RobotModelHyrodyn::getActiveContacts());
}
base::NamedVector<base::JointLimitRange> RobotModelHyrodyn::jointLimits2(){
    return fromJointLimits(wbc::RobotModelHyrodyn::jointLimits());
}
wbc_py::RobotModelConfig RobotModelHyrodyn::getRobotModelConfig(){
    return fromRobotModelConfig(wbc::RobotModelHyrodyn::getRobotModelConfig());
}


}

BOOST_PYTHON_MODULE(robot_model_hyrodyn){

    np::initialize();

    py::class_<wbc_py::RobotModelHyrodyn>("RobotModelHyrodyn")
            .def("configure",               &wbc_py::RobotModelHyrodyn::configure)
            .def("update",                  &wbc_py::RobotModelHyrodyn::update)
            .def("update",                  &wbc_py::RobotModelHyrodyn::update2)
            .def("jointState",              &wbc_py::RobotModelHyrodyn::jointState2)
            .def("rigidBodyState",          &wbc_py::RobotModelHyrodyn::rigidBodyState, py::return_value_policy<py::copy_const_reference>())
            .def("spaceJacobian",           &wbc_py::RobotModelHyrodyn::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("bodyJacobian",            &wbc_py::RobotModelHyrodyn::spaceJacobian, py::return_value_policy<py::copy_const_reference>())
            .def("spatialAccelerationBias", &wbc_py::RobotModelHyrodyn::spatialAccelerationBias, py::return_value_policy<py::copy_const_reference>())
            .def("jacobianDot",             &wbc_py::RobotModelHyrodyn::jacobianDot, py::return_value_policy<py::copy_const_reference>())
            .def("jointSpaceInertiaMatrix", &wbc_py::RobotModelHyrodyn::jointSpaceInertiaMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("biasForces",              &wbc_py::RobotModelHyrodyn::biasForces, py::return_value_policy<py::copy_const_reference>())
            .def("jointNames",              &wbc_py::RobotModelHyrodyn::jointNames, py::return_value_policy<py::copy_const_reference>())
            .def("actuatedJointNames",      &wbc_py::RobotModelHyrodyn::actuatedJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("independentJointNames",   &wbc_py::RobotModelHyrodyn::independentJointNames, py::return_value_policy<py::copy_const_reference>())
            .def("jointIndex",              &wbc_py::RobotModelHyrodyn::jointIndex)
            .def("baseFrame",               &wbc_py::RobotModelHyrodyn::baseFrame, py::return_value_policy<py::copy_const_reference>())
            .def("jointLimits",             &wbc_py::RobotModelHyrodyn::jointLimits2)
            .def("selectionMatrix",         &wbc_py::RobotModelHyrodyn::selectionMatrix, py::return_value_policy<py::copy_const_reference>())
            .def("hasLink",                 &wbc_py::RobotModelHyrodyn::hasLink)
            .def("hasJoint",                &wbc_py::RobotModelHyrodyn::hasJoint)
            .def("hasActuatedJoint",        &wbc_py::RobotModelHyrodyn::hasActuatedJoint)
            .def("centerOfMass",            &wbc_py::RobotModelHyrodyn::centerOfMass, py::return_value_policy<py::copy_const_reference>())
            .def("setActiveContacts",       &wbc_py::RobotModelHyrodyn::setActiveContacts)
            .def("getActiveContacts",       &wbc_py::RobotModelHyrodyn::getActiveContacts2)
            .def("noOfJoints",              &wbc_py::RobotModelHyrodyn::noOfJoints)
            .def("noOfActuatedJoints",      &wbc_py::RobotModelHyrodyn::noOfActuatedJoints)
            .def("setGravityVector",        &wbc_py::RobotModelHyrodyn::setGravityVector)
            .def("floatingBaseState",       &wbc_py::RobotModelHyrodyn::floatingBaseState, py::return_value_policy<py::copy_const_reference>())
            .def("getRobotModelConfig",     &wbc_py::RobotModelHyrodyn::getRobotModelConfig);

}


