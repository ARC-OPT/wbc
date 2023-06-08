#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "acceleration_scene_tsid.hpp"

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

AccelerationSceneTSID::AccelerationSceneTSID(std::shared_ptr<RobotModelRBDL> robot_model, std::shared_ptr<QPOASESSolver> solver, const double dt) :
    wbc::AccelerationSceneTSID(robot_model, solver, dt){
}
void AccelerationSceneTSID::setJointReference(const std::string& task_name, const base::NamedVector<base::JointState>& ref){
    wbc::AccelerationSceneTSID::setReference(task_name, tobaseSamplesJoints(ref));
}
void AccelerationSceneTSID::setCartReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::AccelerationSceneTSID::setReference(task_name, ref);
}
void AccelerationSceneTSID::setJointWeights(const base::NamedVector<double> &weights){
    wbc::AccelerationSceneTSID::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> AccelerationSceneTSID::getJointWeights2(){
    return toNamedVector(wbc::AccelerationSceneTSID::getJointWeights());
}
base::NamedVector<double> AccelerationSceneTSID::getActuatedJointWeights2(){
    return toNamedVector(wbc::AccelerationSceneTSID::getActuatedJointWeights());
}
base::NamedVector<base::JointState> AccelerationSceneTSID::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::AccelerationSceneTSID::solve(hqp));
}
base::NamedVector<wbc::TaskStatus> AccelerationSceneTSID::updateTasksStatus2(){
    return toNamedVector(wbc::AccelerationSceneTSID::updateTasksStatus());
}

}

BOOST_PYTHON_MODULE(acceleration_scene_tsid){

    np::initialize();

    py::class_<wbc_py::AccelerationSceneTSID>("AccelerationSceneTSID", py::init<std::shared_ptr<wbc_py::RobotModelRBDL>,std::shared_ptr<wbc_py::QPOASESSolver>,const double>())
            .def("configure",    &wbc_py::AccelerationSceneTSID::configure)
            .def("update",       &wbc_py::AccelerationSceneTSID::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::AccelerationSceneTSID::solve2)
            .def("setReference", &wbc_py::AccelerationSceneTSID::setJointReference)
            .def("setReference", &wbc_py::AccelerationSceneTSID::setCartReference)
            .def("setTaskWeights",   &wbc_py::AccelerationSceneTSID::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::AccelerationSceneTSID::setTaskActivation)
            .def("getTasksStatus",   &wbc_py::AccelerationSceneTSID::getTasksStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::AccelerationSceneTSID::getNTaskVariablesPerPrio)
            .def("hasTask",   &wbc_py::AccelerationSceneTSID::hasTask)
            .def("updateTasksStatus",   &wbc_py::AccelerationSceneTSID::updateTasksStatus2)
            .def("getHierarchicalQP",   &wbc_py::AccelerationSceneTSID::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::AccelerationSceneTSID::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::AccelerationSceneTSID::setJointWeights)
            .def("getJointWeights",   &wbc_py::AccelerationSceneTSID::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::AccelerationSceneTSID::getActuatedJointWeights2);
}
