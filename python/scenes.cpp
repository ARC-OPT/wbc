#include "eigen_conversion.h"
#include "base_types_conversion.h"
#include "std_vector_conversion.h"
#include "scenes.hpp"

namespace wbc_py {

wbc::JointWeights toJointWeights(const base::NamedVector<double> &weights){
    wbc::JointWeights weights_out;
    weights_out.elements = weights.elements;
    weights_out.names = weights.names;
    return weights_out;
}

base::NamedVector<double> toNamedVector(const wbc::JointWeights &weights){
    base::NamedVector<double> weights_out;
    weights_out.elements = weights.elements;
    weights_out.names = weights.names;
    return weights_out;
}

base::NamedVector<wbc::ConstraintStatus> toNamedVector(const wbc::ConstraintsStatus& status_in){
    base::NamedVector<wbc::ConstraintStatus> status_out;
    status_out.elements = status_in.elements;
    status_out.names = status_in.names;
    return status_out;
}

VelocityScene::VelocityScene(std::shared_ptr<RobotModelKDL> robot_model, std::shared_ptr<HierarchicalLSSolver> solver) :
    wbc::VelocityScene(robot_model, solver){
}
VelocityScene::VelocityScene(std::shared_ptr<RobotModelKDL> robot_model, std::shared_ptr<QPOASESSolver> solver) :
    wbc::VelocityScene(robot_model, solver){
}
void VelocityScene::setJointReference(const std::string& constraint_name, const base::NamedVector<base::JointState>& ref){
    wbc::VelocityScene::setReference(constraint_name, tobaseSamplesJoints(ref));
}
void VelocityScene::setCartReference(const std::string& constraint_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::VelocityScene::setReference(constraint_name, ref);
}
void VelocityScene::setJointWeights(const base::NamedVector<double> &weights){
    wbc::VelocityScene::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> VelocityScene::getJointWeights2(){
    return toNamedVector(wbc::VelocityScene::getJointWeights());
}
base::NamedVector<base::JointState> VelocityScene::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::VelocityScene::solve(hqp));
}
base::NamedVector<wbc::ConstraintStatus> VelocityScene::updateConstraintsStatus2(){
    return toNamedVector(wbc::VelocityScene::updateConstraintsStatus());
}

AccelerationSceneTSID::AccelerationSceneTSID(std::shared_ptr<RobotModelHyrodyn> robot_model, std::shared_ptr<QPOASESSolver> solver) :
    wbc::AccelerationSceneTSID(robot_model, solver){
}
void AccelerationSceneTSID::setJointReference(const std::string& constraint_name, const base::NamedVector<base::JointState>& ref){
    wbc::AccelerationSceneTSID::setReference(constraint_name, tobaseSamplesJoints(ref));
}
void AccelerationSceneTSID::setCartReference(const std::string& constraint_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::AccelerationSceneTSID::setReference(constraint_name, ref);
}
void AccelerationSceneTSID::setJointWeights(const base::NamedVector<double> &weights){
    wbc::AccelerationSceneTSID::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> AccelerationSceneTSID::getJointWeights2(){
    return toNamedVector(wbc::AccelerationSceneTSID::getJointWeights());
}
base::NamedVector<base::JointState> AccelerationSceneTSID::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::AccelerationSceneTSID::solve(hqp));
}
base::NamedVector<wbc::ConstraintStatus> AccelerationSceneTSID::updateConstraintsStatus2(){
    return toNamedVector(wbc::AccelerationSceneTSID::updateConstraintsStatus());
}
}

BOOST_PYTHON_MODULE(scenes){

    np::initialize();

    py::class_<wbc_py::VelocityScene>("VelocityScene", py::init<std::shared_ptr<wbc_py::RobotModelKDL>, std::shared_ptr<wbc_py::QPOASESSolver>>())
            .def(py::init<std::shared_ptr<wbc_py::RobotModelKDL>,std::shared_ptr<wbc_py::HierarchicalLSSolver>>())
            .def("configure",    &wbc_py::VelocityScene::configure)
            .def("update",       &wbc_py::VelocityScene::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::VelocityScene::solve2)
            .def("setReference", &wbc_py::VelocityScene::setJointReference)
            .def("setReference", &wbc_py::VelocityScene::setCartReference)
            .def("setTaskWeights",   &wbc_py::VelocityScene::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::VelocityScene::setTaskActivation)
            .def("getConstraintsStatus",   &wbc_py::VelocityScene::getConstraintsStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::VelocityScene::getNConstraintVariablesPerPrio)
            .def("hasConstraint",   &wbc_py::VelocityScene::hasConstraint)
            .def("updateConstraintsStatus",   &wbc_py::VelocityScene::updateConstraintsStatus2)
            .def("getHierarchicalQP",   &wbc_py::VelocityScene::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::VelocityScene::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::VelocityScene::setJointWeights)
            .def("getJointWeights",   &wbc_py::VelocityScene::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::VelocityScene::getActuatedJointWeights,  py::return_value_policy<py::copy_const_reference>());


    py::class_<wbc_py::AccelerationSceneTSID>("AccelerationSceneTSID", py::init<std::shared_ptr<wbc_py::RobotModelHyrodyn>, std::shared_ptr<wbc_py::QPOASESSolver>>())
            .def("configure",    &wbc_py::AccelerationSceneTSID::configure)
            .def("update",       &wbc_py::AccelerationSceneTSID::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::AccelerationSceneTSID::solve2)
            .def("setReference", &wbc_py::AccelerationSceneTSID::setJointReference)
            .def("setReference", &wbc_py::AccelerationSceneTSID::setCartReference)
            .def("setTaskWeights",   &wbc_py::AccelerationSceneTSID::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::AccelerationSceneTSID::setTaskActivation)
            .def("getConstraintsStatus",   &wbc_py::AccelerationSceneTSID::getConstraintsStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::AccelerationSceneTSID::getNConstraintVariablesPerPrio)
            .def("hasConstraint",   &wbc_py::AccelerationSceneTSID::hasConstraint)
            .def("updateConstraintsStatus",   &wbc_py::AccelerationSceneTSID::updateConstraintsStatus2)
            .def("getHierarchicalQP",   &wbc_py::AccelerationSceneTSID::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::AccelerationSceneTSID::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::AccelerationSceneTSID::setJointWeights)
            .def("getJointWeights",   &wbc_py::AccelerationSceneTSID::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::AccelerationSceneTSID::getActuatedJointWeights,  py::return_value_policy<py::copy_const_reference>())
            .def("getContactWrenches",   &wbc_py::AccelerationSceneTSID::getActuatedJointWeights,  py::return_value_policy<py::copy_const_reference>());

}


