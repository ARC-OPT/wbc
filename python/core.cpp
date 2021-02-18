#include "eigen_conversion.h"
#include "base_types_conversion.h"
#include "std_vector_conversion.h"
#include "core/RobotModelConfig.hpp"
#include "core/ConstraintConfig.hpp"
#include "core/ConstraintStatus.hpp"
#include "core/QuadraticProgram.hpp"
#include <base/samples/Joints.hpp>
#include <boost/python/enum.hpp>

BOOST_PYTHON_MODULE(core){

    np::initialize();

    pygen::convertMatrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign>>();
    pygen::convertVector<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>>();
    pygen::convertStdVector<std::vector<std::string>>();
    pygen::convertStdVector<std::vector<double>>();
    pygen::convertStdVector<std::vector<wbc::QuadraticProgram>>();
    pygen::convertStdVector<std::vector<base::JointState>>();
    pygen::convertStdVector<std::vector<wbc::ConstraintConfig>>();
    pygen::convertVector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign>>();
    pygen::convertVector<Eigen::Matrix<double, 6, 1, Eigen::DontAlign>>();
    pygen::convertTransform<Eigen::Transform<double, 3, Eigen::DontAlign>>();
    pygen::convertQuaternion<Eigen::Quaternion<double, Eigen::DontAlign>>();
    pygen::convertStdVector<std::vector<wbc::ConstraintStatus>>();

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

    py::class_<base::samples::RigidBodyStateSE3>("RigidBodyStateSE3")
            .def_readwrite("pose", &base::RigidBodyStateSE3::pose)
            .def_readwrite("twist", &base::RigidBodyStateSE3::twist)
            .def_readwrite("acceleration", &base::RigidBodyStateSE3::acceleration)
            .def_readwrite("wrench", &base::RigidBodyStateSE3::wrench);

   py::class_<base::JointState>("JointState")
            .def_readwrite("position", &base::JointState::position)
            .def_readwrite("speed", &base::JointState::speed)
            .def_readwrite("acceleration", &base::JointState::acceleration)
            .def_readwrite("effort", &base::JointState::speed)
            .def_readwrite("raw", &base::JointState::raw);

   py::class_<base::NamedVector<base::JointState>>("Joints")
            .add_property("names",
                py::make_getter(&base::NamedVector<base::JointState>::names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointState>::names))
            .add_property("elements",
                py::make_getter(&base::NamedVector<base::JointState>::elements, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&base::NamedVector<base::JointState>::elements));

   py::class_<wbc::RobotModelConfig>("RobotModelConfig")
            .def_readwrite("file",                 &wbc::RobotModelConfig::file)
            .def_readwrite("submechanism_file",    &wbc::RobotModelConfig::submechanism_file)
            .add_property("joint_names",
                py::make_getter(&wbc::RobotModelConfig::joint_names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::RobotModelConfig::joint_names))
            .add_property("actuated_joint_names",
                py::make_getter(&wbc::RobotModelConfig::actuated_joint_names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::RobotModelConfig::actuated_joint_names))
            .def_readwrite("floating_base",        &wbc::RobotModelConfig::floating_base)
            .def_readwrite("world_frame_id",       &wbc::RobotModelConfig::world_frame_id)
            .def_readwrite("floating_base_state",  &wbc::RobotModelConfig::floating_base_state)
            .add_property("contact_points",
                py::make_getter(&wbc::RobotModelConfig::contact_points, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::RobotModelConfig::contact_points));

   py::enum_<wbc::ConstraintType>("ConstraintType")
       .value("unset", wbc::ConstraintType::unset)
       .value("cart", wbc::ConstraintType::cart)
       .value("jnt", wbc::ConstraintType::jnt);

   py::class_<wbc::ConstraintConfig>("ConstraintConfig")
            .def_readwrite("name",       &wbc::ConstraintConfig::name)
            .def_readwrite("type",       &wbc::ConstraintConfig::type)
            .def_readwrite("priority",   &wbc::ConstraintConfig::priority)
            .add_property("weights",
                py::make_getter(&wbc::ConstraintConfig::weights, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::ConstraintConfig::weights))
            .def_readwrite("activation", &wbc::ConstraintConfig::activation)
            .def_readwrite("timeout",    &wbc::ConstraintConfig::timeout)
            .add_property("joint_names",
                py::make_getter(&wbc::ConstraintConfig::joint_names, py::return_value_policy<py::copy_non_const_reference>()),
                py::make_setter(&wbc::ConstraintConfig::joint_names))
            .def_readwrite("root",       &wbc::ConstraintConfig::root)
            .def_readwrite("tip",        &wbc::ConstraintConfig::tip)
            .def_readwrite("ref_frame",  &wbc::ConstraintConfig::ref_frame);

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

   py::class_<wbc::ConstraintStatus>("ConstraintStatus")
       .def_readwrite("config",  &wbc::ConstraintStatus::config)
       .def_readwrite("activation",  &wbc::ConstraintStatus::activation)
       .def_readwrite("timeout",  &wbc::ConstraintStatus::timeout)
       .add_property("weights",
                     py::make_getter(&wbc::ConstraintStatus::weights, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::ConstraintStatus::weights))
       .add_property("y_ref",
                     py::make_getter(&wbc::ConstraintStatus::y_ref, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::ConstraintStatus::y_ref))
       .add_property("y_solution",
                     py::make_getter(&wbc::ConstraintStatus::y_solution, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::ConstraintStatus::y_solution))
       .add_property("y",
                     py::make_getter(&wbc::ConstraintStatus::y, py::return_value_policy<py::copy_non_const_reference>()),
                     py::make_setter(&wbc::ConstraintStatus::y));

   py::class_<base::NamedVector<wbc::ConstraintStatus>>("ConstraintsStatus")
           .add_property("names",
               py::make_getter(&wbc::ConstraintsStatus::names, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::ConstraintsStatus::names))
           .add_property("elements",
               py::make_getter(&wbc::ConstraintsStatus::elements, py::return_value_policy<py::copy_non_const_reference>()),
               py::make_setter(&wbc::ConstraintsStatus::elements));
}

