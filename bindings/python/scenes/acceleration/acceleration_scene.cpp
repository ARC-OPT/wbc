#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "acceleration_scene.hpp"

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

AccelerationScene::AccelerationScene(std::shared_ptr<RobotModelRBDL> robot_model, std::shared_ptr<QPOASESSolver> solver, const double dt) :
    wbc::AccelerationScene(robot_model, solver, dt){
}
void AccelerationScene::setJointReference(const std::string& task_name, const base::NamedVector<base::JointState>& ref){
    wbc::AccelerationScene::setReference(task_name, tobaseSamplesJoints(ref));
}
void AccelerationScene::setCartReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::AccelerationScene::setReference(task_name, ref);
}
void AccelerationScene::setJointWeights(const base::NamedVector<double> &weights){
    wbc::AccelerationScene::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> AccelerationScene::getJointWeights2(){
    return toNamedVector(wbc::AccelerationScene::getJointWeights());
}
base::NamedVector<double> AccelerationScene::getActuatedJointWeights2(){
    return toNamedVector(wbc::AccelerationScene::getActuatedJointWeights());
}
base::NamedVector<base::JointState> AccelerationScene::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::AccelerationScene::solve(hqp));
}
base::NamedVector<wbc::TaskStatus> AccelerationScene::updateTasksStatus2(){
    return toNamedVector(wbc::AccelerationScene::updateTasksStatus());
}

}

BOOST_PYTHON_MODULE(acceleration_scene){

    np::initialize();

    py::class_<wbc_py::AccelerationScene>("AccelerationScene", py::init<std::shared_ptr<wbc_py::RobotModelRBDL>,std::shared_ptr<wbc_py::QPOASESSolver>,const double>())
            .def("configure",    &wbc_py::AccelerationScene::configure)
            .def("update",       &wbc_py::AccelerationScene::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::AccelerationScene::solve2)
            .def("setReference", &wbc_py::AccelerationScene::setJointReference)
            .def("setReference", &wbc_py::AccelerationScene::setCartReference)
            .def("setTaskWeights",   &wbc_py::AccelerationScene::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::AccelerationScene::setTaskActivation)
            .def("getTasksStatus",   &wbc_py::AccelerationScene::getTasksStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::AccelerationScene::getNTaskVariablesPerPrio)
            .def("hasTask",   &wbc_py::AccelerationScene::hasTask)
            .def("updateTasksStatus",   &wbc_py::AccelerationScene::updateTasksStatus2)
            .def("getHierarchicalQP",   &wbc_py::AccelerationScene::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::AccelerationScene::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::AccelerationScene::setJointWeights)
            .def("getJointWeights",   &wbc_py::AccelerationScene::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::AccelerationScene::getActuatedJointWeights2);
}
