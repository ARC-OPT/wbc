#include "../eigen_conversion.h"
#include "../std_vector_conversion.h"
#include "../wbc_types_conversions.h"
#include "core/RobotModelConfig.hpp"
#include "core/TaskConfig.hpp"
#include "core/TaskStatus.hpp"
#include "core/QuadraticProgram.hpp"
#include <base/JointLimits.hpp>
#include <boost/python/enum.hpp>

BOOST_PYTHON_MODULE(core){

    np::initialize();

    pygen::convertMatrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign>>();
    pygen::convertVector<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>>();
    pygen::convertStdVector<std::vector<std::string>>();
    pygen::convertStdVector<std::vector<double>>();
    pygen::convertStdVector<std::vector<int>>();
    pygen::convertStdVector<std::vector<wbc::QuadraticProgram>>();
    pygen::convertStdVector<std::vector<base::JointState>>();
    pygen::convertStdVector<std::vector<base::JointLimitRange>>();
    pygen::convertStdVector<std::vector<base::Wrench>>();
    pygen::convertStdVector<std::vector<wbc::TaskConfig>>();
    pygen::convertVector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>();
    pygen::convertVector<Eigen::Matrix<double, 6, 1, Eigen::DontAlign>>();
    pygen::convertTransform<Eigen::Transform<double, 3, Eigen::DontAlign>>();
    pygen::convertQuaternion<Eigen::Quaternion<double, Eigen::DontAlign>>();
    pygen::convertStdVector<std::vector<wbc::TaskStatus>>();

    py::class_<base::Pose>("Pose")
            .add_property("position",
                py::make_getter(&base::Pose::position, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Pose::position))
            .add_property("orientation",
                py::make_getter(&base::Pose::orientation, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Pose::orientation))
            .def("toVector6d", &base::Pose::toVector6d)
            .def("toTransform", &base::Pose::toTransform)
            .def("fromTransform", &base::Pose::fromTransform);

    py::class_<base::Twist>("Twist")
            .add_property("linear",
                py::make_getter(&base::Twist::linear,py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Twist::linear))
            .add_property("angular",
                py::make_getter(&base::Twist::angular, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Twist::angular));

    py::class_<base::Acceleration>("Acceleration")
            .add_property("linear",
                py::make_getter(&base::Acceleration::linear,py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Acceleration::linear))
            .add_property("angular",
                py::make_getter(&base::Acceleration::angular, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Acceleration::angular));

    py::class_<base::Wrench>("Wrench")
            .add_property("force",
                py::make_getter(&base::Wrench::force,py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Wrench::force))
            .add_property("torque",
                py::make_getter(&base::Wrench::torque, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::Wrench::torque));

    py::class_<base::NamedVector<base::Wrench>>("Wrenches")
            .add_property("names",
                py::make_getter(&base::NamedVector<base::Wrench>::names,py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::Wrench>::names))
            .add_property("elements",
                py::make_getter(&base::NamedVector<base::Wrench>::elements, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::Wrench>::elements));

    py::class_<base::RigidBodyStateSE3>("baseRigidBodyStateSE3")
            .def_readwrite("pose", &base::RigidBodyStateSE3::pose)
            .def_readwrite("twist", &base::RigidBodyStateSE3::twist)
            .def_readwrite("acceleration", &base::RigidBodyStateSE3::acceleration)
            .def_readwrite("wrench", &base::RigidBodyStateSE3::wrench);

    py::class_<base::Time>("Time")
            .def_readwrite("microseconds", &base::Time::microseconds);

    py::class_<base::samples::RigidBodyStateSE3>("RigidBodyStateSE3")
            .def_readwrite("time", &base::samples::RigidBodyStateSE3::time)
            .def_readwrite("pose", &base::RigidBodyStateSE3::pose)
            .def_readwrite("twist", &base::RigidBodyStateSE3::twist)
            .def_readwrite("acceleration", &base::RigidBodyStateSE3::acceleration)
            .def_readwrite("wrench", &base::RigidBodyStateSE3::wrench);

   py::class_<base::JointState>("JointState")
            .def_readwrite("position", &base::JointState::position)
            .def_readwrite("speed", &base::JointState::speed)
            .def_readwrite("acceleration", &base::JointState::acceleration)
            .def_readwrite("effort", &base::JointState::effort)
            .def_readwrite("raw", &base::JointState::raw);

   py::class_<base::NamedVector<base::JointState>>("Joints")
            .add_property("names",
                py::make_getter(&base::NamedVector<base::JointState>::names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointState>::names))
            .add_property("elements",
                py::make_getter(&base::NamedVector<base::JointState>::elements, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointState>::elements));

   py::class_<base::JointLimitRange>("JointLimitRange")
           .add_property("min",
               py::make_getter(&base::JointLimitRange::min, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&base::JointLimitRange::min))
           .add_property("max",
                         py::make_getter(&base::JointLimitRange::max, py::return_value_policy<py::copy_non_const_reference>()),
                         py::make_setter(&base::JointLimitRange::max));

   py::class_<base::NamedVector<base::JointLimitRange>>("JointLimits")
            .add_property("names",
                py::make_getter(&base::NamedVector<base::JointLimitRange>::names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointLimitRange>::names))
            .add_property("elements",
                py::make_getter(&base::NamedVector<base::JointLimitRange>::elements, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointLimitRange>::elements));

   py::class_<wbc_py::RobotModelConfig>("RobotModelConfig")
            .def_readwrite("file",                 &wbc::RobotModelConfig::file)
            .def_readwrite("submechanism_file",    &wbc::RobotModelConfig::submechanism_file)
            .def_readwrite("floating_base",        &wbc::RobotModelConfig::floating_base)
            .def_readwrite("floating_base",        &wbc::RobotModelConfig::floating_base)
            .add_property("contact_points",        &wbc_py::RobotModelConfig::getActiveContacts, &wbc_py::RobotModelConfig::setActiveContacts);

   py::enum_<wbc::TaskType>("TaskType")
       .value("unset", wbc::TaskType::unset)
       .value("cart", wbc::TaskType::cart)
       .value("jnt", wbc::TaskType::jnt);

   py::class_<wbc::TaskConfig>("TaskConfig")
            .def_readwrite("name",       &wbc::TaskConfig::name)
            .def_readwrite("type",       &wbc::TaskConfig::type)
            .def_readwrite("priority",   &wbc::TaskConfig::priority)
            .add_property("weights",
                py::make_getter(&wbc::TaskConfig::weights, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::TaskConfig::weights))
            .def_readwrite("activation", &wbc::TaskConfig::activation)
            .def_readwrite("timeout",    &wbc::TaskConfig::timeout)
            .add_property("joint_names",
                py::make_getter(&wbc::TaskConfig::joint_names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::TaskConfig::joint_names))
            .def_readwrite("root",       &wbc::TaskConfig::root)
            .def_readwrite("tip",        &wbc::TaskConfig::tip)
            .def_readwrite("ref_frame",  &wbc::TaskConfig::ref_frame);

   py::class_<wbc::QuadraticProgram>("QuadraticProgram")
           .add_property("A",
               py::make_getter(&wbc::QuadraticProgram::A, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::A))
           .add_property("g",
               py::make_getter(&wbc::QuadraticProgram::g, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::g))
           .add_property("lower_x",
               py::make_getter(&wbc::QuadraticProgram::lower_x, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::lower_x))
           .add_property("upper_x",
               py::make_getter(&wbc::QuadraticProgram::upper_x, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::upper_x))
           .add_property("lower_y",
               py::make_getter(&wbc::QuadraticProgram::lower_y, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::lower_y))
           .add_property("upper_y",
               py::make_getter(&wbc::QuadraticProgram::upper_y, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::upper_y))
           .add_property("H",
               py::make_getter(&wbc::QuadraticProgram::H, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::H))
           .add_property("Wy",
               py::make_getter(&wbc::QuadraticProgram::Wy, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::QuadraticProgram::Wy))
           .def_readwrite("nc",  &wbc::QuadraticProgram::nc)
           .def_readwrite("nq",  &wbc::QuadraticProgram::nq)
           .def("resize",  &wbc::QuadraticProgram::resize);

   py::class_<wbc::HierarchicalQP>("HierarchicalQP")
       .add_property("prios",
           py::make_getter(&wbc::HierarchicalQP::prios, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&wbc::HierarchicalQP::prios))
       .add_property("Wq",
           py::make_getter(&wbc::HierarchicalQP::Wq, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&wbc::HierarchicalQP::Wq))
       .def("resize",  &wbc::HierarchicalQP::resize);

   py::class_<base::NamedVector<double>>("JointWeights")
       .add_property("names",
           py::make_getter(&wbc::JointWeights::names, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&wbc::JointWeights::names))
       .add_property("elements",
           py::make_getter(&wbc::JointWeights::elements, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&wbc::JointWeights::elements));

   py::class_<wbc::ActiveContact>("ActiveContact")
           .def_readwrite("active", &wbc::ActiveContact::active)
           .def_readwrite("mu",     &wbc::ActiveContact::mu);

   py::class_<wbc::ActiveContact>("ActiveContacts")
       .add_property("names",
           py::make_getter(&base::NamedVector<int>::names, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&base::NamedVector<int>::names))
       .add_property("elements",
           py::make_getter(&base::NamedVector<wbc::ActiveContact>::elements, py::return_value_policy<py::copy_non_const_reference>()),
           py::make_setter(&base::NamedVector<wbc::ActiveContact>::elements));

   py::class_<wbc::TaskStatus>("TaskStatus")
       .def_readwrite("config",  &wbc::TaskStatus::config)
       .def_readwrite("activation",  &wbc::TaskStatus::activation)
       .def_readwrite("timeout",  &wbc::TaskStatus::timeout)
       .add_property("weights",
                     py::make_getter(&wbc::TaskStatus::weights, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::TaskStatus::weights))
       .add_property("y_ref",
                     py::make_getter(&wbc::TaskStatus::y_ref, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::TaskStatus::y_ref))
       .add_property("y_solution",
                     py::make_getter(&wbc::TaskStatus::y_solution, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::TaskStatus::y_solution))
       .add_property("y",
                     py::make_getter(&wbc::TaskStatus::y, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::TaskStatus::y));

   py::class_<base::NamedVector<wbc::TaskStatus>>("TasksStatus")
           .add_property("names",
               py::make_getter(&wbc::TasksStatus::names, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::TasksStatus::names))
           .add_property("elements",
               py::make_getter(&wbc::TasksStatus::elements, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::TasksStatus::elements));
}

