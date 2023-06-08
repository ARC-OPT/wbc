#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "velocity_scene.hpp"

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

base::NamedVector<wbc::TaskStatus> toNamedVector(const wbc::TasksStatus& status_in){
    base::NamedVector<wbc::TaskStatus> status_out;
    status_out.elements = status_in.elements;
    status_out.names = status_in.names;
    return status_out;
}

base::NamedVector<base::Wrench> toNamedVector(const base::samples::Wrenches& wrenches_in){
    base::NamedVector<base::Wrench> wrenches_out;
    wrenches_out.elements = wrenches_in.elements;
    wrenches_out.names = wrenches_in.names;
    return wrenches_out;
}

VelocityScene::VelocityScene(std::shared_ptr<RobotModelRBDL> robot_model, std::shared_ptr<HierarchicalLSSolver> solver, const double dt) :
    wbc::VelocityScene(robot_model, solver, dt){
}
void VelocityScene::setJointReference(const std::string& task_name, const base::NamedVector<base::JointState>& ref){
    wbc::VelocityScene::setReference(task_name, tobaseSamplesJoints(ref));
}
void VelocityScene::setCartReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::VelocityScene::setReference(task_name, ref);
}
void VelocityScene::setJointWeights(const base::NamedVector<double> &weights){
    wbc::VelocityScene::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> VelocityScene::getJointWeights2(){
    return toNamedVector(wbc::VelocityScene::getJointWeights());
}
base::NamedVector<double> VelocityScene::getActuatedJointWeights2(){
    return toNamedVector(wbc::VelocityScene::getActuatedJointWeights());
}
base::NamedVector<base::JointState> VelocityScene::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::VelocityScene::solve(hqp));
}
base::NamedVector<wbc::TaskStatus> VelocityScene::updateTasksStatus2(){
    return toNamedVector(wbc::VelocityScene::updateTasksStatus());
}

}

BOOST_PYTHON_MODULE(velocity_scene){

    np::initialize();

    py::class_<wbc_py::VelocityScene>("VelocityScene", py::init<std::shared_ptr<wbc_py::RobotModelRBDL>,std::shared_ptr<wbc_py::HierarchicalLSSolver>,const double>())
            .def("configure",    &wbc_py::VelocityScene::configure)
            .def("update",       &wbc_py::VelocityScene::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::VelocityScene::solve2)
            .def("setReference", &wbc_py::VelocityScene::setJointReference)
            .def("setReference", &wbc_py::VelocityScene::setCartReference)
            .def("setTaskWeights",   &wbc_py::VelocityScene::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::VelocityScene::setTaskActivation)
            .def("getTasksStatus",   &wbc_py::VelocityScene::getTasksStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::VelocityScene::getNTaskVariablesPerPrio)
            .def("hasTask",   &wbc_py::VelocityScene::hasTask)
            .def("updateTasksStatus",   &wbc_py::VelocityScene::updateTasksStatus2)
            .def("getHierarchicalQP",   &wbc_py::VelocityScene::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::VelocityScene::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::VelocityScene::setJointWeights)
            .def("getJointWeights",   &wbc_py::VelocityScene::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::VelocityScene::getActuatedJointWeights2);
}
