#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "acceleration_scene_reduced_tsid.hpp"

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

AccelerationSceneReducedTSID::AccelerationSceneReducedTSID(std::shared_ptr<RobotModelRBDL> robot_model, std::shared_ptr<QPOASESSolver> solver, const double dt) :
    wbc::AccelerationSceneReducedTSID(robot_model, solver, dt){
}
void AccelerationSceneReducedTSID::setJointReference(const std::string& task_name, const base::NamedVector<base::JointState>& ref){
    wbc::AccelerationSceneReducedTSID::setReference(task_name, tobaseSamplesJoints(ref));
}
void AccelerationSceneReducedTSID::setCartReference(const std::string& task_name, const base::samples::RigidBodyStateSE3& ref){
    wbc::AccelerationSceneReducedTSID::setReference(task_name, ref);
}
void AccelerationSceneReducedTSID::setJointWeights(const base::NamedVector<double> &weights){
    wbc::AccelerationSceneReducedTSID::setJointWeights(toJointWeights(weights));
}
base::NamedVector<double> AccelerationSceneReducedTSID::getJointWeights2(){
    return toNamedVector(wbc::AccelerationSceneReducedTSID::getJointWeights());
}
base::NamedVector<double> AccelerationSceneReducedTSID::getActuatedJointWeights2(){
    return toNamedVector(wbc::AccelerationSceneReducedTSID::getActuatedJointWeights());
}
base::NamedVector<base::JointState> AccelerationSceneReducedTSID::solve2(const wbc::HierarchicalQP &hqp){
    return toNamedVector(wbc::AccelerationSceneReducedTSID::solve(hqp));
}
base::NamedVector<wbc::TaskStatus> AccelerationSceneReducedTSID::updateTasksStatus2(){
    return toNamedVector(wbc::AccelerationSceneReducedTSID::updateTasksStatus());
}

}

BOOST_PYTHON_MODULE(acceleration_scene_reduced_tsid){

    np::initialize();

    py::class_<wbc_py::AccelerationSceneReducedTSID>("AccelerationSceneReducedTSID", py::init<std::shared_ptr<wbc_py::RobotModelRBDL>,std::shared_ptr<wbc_py::QPOASESSolver>,const double>())
            .def("configure",    &wbc_py::AccelerationSceneReducedTSID::configure)
            .def("update",       &wbc_py::AccelerationSceneReducedTSID::update, py::return_value_policy<py::copy_const_reference>())
            .def("solve",        &wbc_py::AccelerationSceneReducedTSID::solve2)
            .def("setReference", &wbc_py::AccelerationSceneReducedTSID::setJointReference)
            .def("setReference", &wbc_py::AccelerationSceneReducedTSID::setCartReference)
            .def("setTaskWeights",   &wbc_py::AccelerationSceneReducedTSID::setTaskWeights)
            .def("setTaskActivation",   &wbc_py::AccelerationSceneReducedTSID::setTaskActivation)
            .def("getTasksStatus",   &wbc_py::AccelerationSceneReducedTSID::getTasksStatus,  py::return_value_policy<py::copy_const_reference>())
            .def("getNConstraintVariablesPerPrio",   &wbc_py::AccelerationSceneReducedTSID::getNTaskVariablesPerPrio)
            .def("hasTask",   &wbc_py::AccelerationSceneReducedTSID::hasTask)
            .def("updateTasksStatus",   &wbc_py::AccelerationSceneReducedTSID::updateTasksStatus2)
            .def("getHierarchicalQP",   &wbc_py::AccelerationSceneReducedTSID::getHierarchicalQP,  py::return_value_policy<py::copy_const_reference>())
            .def("getSolverOutput",   &wbc_py::AccelerationSceneReducedTSID::getSolverOutput,  py::return_value_policy<py::copy_const_reference>())
            .def("setJointWeights",   &wbc_py::AccelerationSceneReducedTSID::setJointWeights)
            .def("getJointWeights",   &wbc_py::AccelerationSceneReducedTSID::getJointWeights2)
            .def("getActuatedJointWeights",   &wbc_py::AccelerationSceneReducedTSID::getActuatedJointWeights2);
}
