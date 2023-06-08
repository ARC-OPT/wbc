#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "velocity_scene_qp.hpp"

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

VelocitySceneQP::VelocitySceneQP(std::shared_ptr<RobotModelRBDL> robot_model, std::shared_ptr<QPOASESSolver> solver, const double dt) :
    wbc::VelocitySceneQP(robot_model, solver, dt){
}
void VelocitySceneQP::setJointReference(const std::string& task_name, const base::NamedVector<base::JointState>& ref){
    wbc::VelocitySceneQP::setReference(task_name, tobaseSamplesJoints(ref));
}
void VelocitySceneQP::setCartReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::VelocitySceneQP::setReference(task_name, ref);
}
void VelocitySceneQP::setJointWeights(const base::NamedVector<double> &weights){
    wbc::VelocitySceneQP::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> VelocitySceneQP::getJointWeights2(){
    return toNamedVector(wbc::VelocitySceneQP::getJointWeights());
}
base::NamedVector<double> VelocitySceneQP::getActuatedJointWeights2(){
    return toNamedVector(wbc::VelocitySceneQP::getActuatedJointWeights());
}
base::NamedVector<base::JointState> VelocitySceneQP::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::VelocitySceneQP::solve(hqp));
}
base::NamedVector<wbc::TaskStatus> VelocitySceneQP::updateTasksStatus2(){
    return toNamedVector(wbc::VelocitySceneQP::updateTasksStatus());
}

}

BOOST_PYTHON_MODULE(velocity_scene_qp){

    np::initialize();

    py::class_<wbc_py::VelocitySceneQP>("VelocitySceneQP", py::init<std::shared_ptr<wbc_py::RobotModelRBDL>,std::shared_ptr<wbc_py::QPOASESSolver>,const double>())
            .def("configure",    &wbc_py::VelocitySceneQP::configure)
            .def("update",       &wbc_py::VelocitySceneQP::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::VelocitySceneQP::solve2)
            .def("setReference", &wbc_py::VelocitySceneQP::setJointReference)
            .def("setReference", &wbc_py::VelocitySceneQP::setCartReference)
            .def("setTaskWeights",   &wbc_py::VelocitySceneQP::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::VelocitySceneQP::setTaskActivation)
            .def("getTasksStatus",   &wbc_py::VelocitySceneQP::getTasksStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::VelocitySceneQP::getNTaskVariablesPerPrio)
            .def("hasTask",   &wbc_py::VelocitySceneQP::hasTask)
            .def("updateTasksStatus",   &wbc_py::VelocitySceneQP::updateTasksStatus2)
            .def("getHierarchicalQP",   &wbc_py::VelocitySceneQP::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::VelocitySceneQP::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::VelocitySceneQP::setJointWeights)
            .def("getJointWeights",   &wbc_py::VelocitySceneQP::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::VelocitySceneQP::getActuatedJointWeights2);
}
