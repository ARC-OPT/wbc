#include "controllers.hpp"

namespace wbc_py {
JointPosPDController::JointPosPDController(const std::vector<std::string> &joint_names) :
    ctrl_lib::JointPosPDController(joint_names){
}
base::NamedVector<base::JointState> JointPosPDController::update(const base::NamedVector<base::JointState> &setpoint,
                                                                 const base::NamedVector<base::JointState> &feedback){
    return toNamedVector(ctrl_lib::JointPosPDController::update(tobaseSamplesJoints(setpoint), tobaseSamplesJoints(feedback)));
}
}

BOOST_PYTHON_MODULE(controllers){

    np::initialize();

    py::class_<ctrl_lib::CartesianPosPDController>("CartesianPosPDController")
            .def("update", &ctrl_lib::CartesianPosPDController::update, py::return_value_policy<py::copy_const_reference>())
            .def("setPGain", &ctrl_lib::CartesianPosPDController::setPGain)
            .def("setDGain", &ctrl_lib::CartesianPosPDController::setDGain)
            .def("setFFGain", &ctrl_lib::CartesianPosPDController::setFFGain)
            .def("setMaxCtrlOutput", &ctrl_lib::CartesianPosPDController::setMaxCtrlOutput)
            .def("setDeadZone", &ctrl_lib::CartesianPosPDController::setDeadZone)
            .def("pGain", &ctrl_lib::CartesianPosPDController::pGain, py::return_value_policy<py::copy_const_reference>())
            .def("dGain", &ctrl_lib::CartesianPosPDController::dGain, py::return_value_policy<py::copy_const_reference>())
            .def("ffGain", &ctrl_lib::CartesianPosPDController::ffGain, py::return_value_policy<py::copy_const_reference>())
            .def("maxCtrlOutput", &ctrl_lib::CartesianPosPDController::maxCtrlOutput, py::return_value_policy<py::copy_const_reference>())
            .def("deadZone", &ctrl_lib::CartesianPosPDController::deadZone, py::return_value_policy<py::copy_const_reference>())
            .def("getControlError", &ctrl_lib::CartesianPosPDController::getControlError);

    py::class_<wbc_py::JointPosPDController>("JointPosPDController", py::init<std::vector<std::string>>())
            .def("update", &wbc_py::JointPosPDController::update)
            .def("setPGain", &wbc_py::JointPosPDController::setPGain)
            .def("setDGain", &wbc_py::JointPosPDController::setDGain)
            .def("setFFGain", &wbc_py::JointPosPDController::setFFGain)
            .def("setMaxCtrlOutput", &wbc_py::JointPosPDController::setMaxCtrlOutput)
            .def("setDeadZone", &wbc_py::JointPosPDController::setDeadZone)
            .def("pGain", &wbc_py::JointPosPDController::pGain, py::return_value_policy<py::copy_const_reference>())
            .def("dGain", &wbc_py::JointPosPDController::dGain, py::return_value_policy<py::copy_const_reference>())
            .def("ffGain", &wbc_py::JointPosPDController::ffGain, py::return_value_policy<py::copy_const_reference>())
            .def("maxCtrlOutput", &wbc_py::JointPosPDController::maxCtrlOutput, py::return_value_policy<py::copy_const_reference>())
            .def("deadZone", &wbc_py::JointPosPDController::deadZone, py::return_value_policy<py::copy_const_reference>())
            .def("getControlError", &wbc_py::JointPosPDController::getControlError);
}
