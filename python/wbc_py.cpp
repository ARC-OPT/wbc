#include "wbc_py.hpp"
#include <boost/python.hpp>

namespace bp = boost::python;

template<class T>
inline
bp::list from_std_vector(const std::vector<T>& v)
{
    bp::object get_iter = bp::iterator<std::vector<T> >();
    bp::object iter = get_iter(v);
    bp::list l(iter);
    return l;
}

template< typename T >
inline
std::vector< T > to_std_vector( const bp::list& iterable )
{
    return std::vector< T >( bp::stl_input_iterator< T >( iterable ),
                             bp::stl_input_iterator< T >( ) );
}

namespace  wbc_py {

base::samples::RigidBodyStateSE3 to_rbs(RigidBodyStateSE3 in){
    base::samples::RigidBodyStateSE3 out;
    for(int i = 0; i < 4; i++)
        out.pose.orientation = base::Quaterniond(bp::extract<double>(in.pose.orientation[0]),
                                                 bp::extract<double>(in.pose.orientation[1]),
                                                 bp::extract<double>(in.pose.orientation[2]),
                                                 bp::extract<double>(in.pose.orientation[0]));
    for(int i = 0; i < 3; i++){
        out.pose.position[i] = bp::extract<double>(in.pose.position[i]);
        out.twist.linear[i] = bp::extract<double>(in.twist.linear[i]);
        out.twist.angular[i] = bp::extract<double>(in.twist.angular[i]);
        out.acceleration.linear[i] = bp::extract<double>(in.acceleration.linear[i]);
        out.acceleration.angular[i] = bp::extract<double>(in.twist.angular[i]);
        out.wrench.force[i] = bp::extract<double>(in.wrench.force[i]);
        out.wrench.torque[i] = bp::extract<double>(in.wrench.torque[i]);
    }
    return out;
}

RigidBodyStateSE3 from_rbs(base::samples::RigidBodyStateSE3 in){
    RigidBodyStateSE3 out;
    for(int i = 0; i < 4; i++)
        out.pose.orientation.append(in.pose.orientation.coeffs()[i]);
    for(int i = 0; i < 3; i++){
        out.pose.position.append(in.pose.position[i]);
        out.twist.linear.append(in.twist.linear[i]);
        out.twist.angular.append(in.twist.angular[i]);
        out.acceleration.linear.append(in.acceleration.linear[i]);
        out.acceleration.angular.append(in.acceleration.angular[i]);
        out.wrench.force.append(in.wrench.force[i]);
        out.wrench.torque.append(in.wrench.torque[i]);
    }
    return out;
}

bool RobotModelHyrodyn::configure(const RobotModelConfig &cfg){
    wbc::RobotModelConfig wbc_cfg(cfg.file,
                                  to_std_vector<std::string>(cfg.actuated_joint_names),
                                  to_std_vector<std::string>(cfg.joint_names),
                                  cfg.floating_base,
                                  cfg.world_frame_id,
                                  to_rbs(cfg.floating_base_state),
                                  to_std_vector<std::string>(cfg.contact_points),
                                  cfg.submechanism_file);
    return robot_model.configure(wbc_cfg);
}

void RobotModelHyrodyn::update(bp::list names,
                               bp::list positions,
                               bp::list velocities,
                               bp::list accelerations,
                               RigidBodyStateSE3 floating_base_state){
    base::samples::Joints joints;
    joints.names = to_std_vector<std::string>(names);
    for(int i = 0; i < bp::len(names); i++){
        base::JointState state;
        state.position     = bp::extract<double>(positions[i]);
        state.speed        = bp::extract<double>(velocities[i]);
        state.acceleration = bp::extract<double>(accelerations[i]);
        joints.elements.push_back(state);
    }
    robot_model.update(joints,to_rbs(floating_base_state));
}

RigidBodyStateSE3 RobotModelHyrodyn::rigidBodyState(const string &root, const string &tip){
    return from_rbs(robot_model.rigidBodyState(root,tip));
}

}

BOOST_PYTHON_MODULE(wbc_py){

    bp::class_<wbc_py::Pose>("Pose")
            .def_readwrite("position", &wbc_py::Pose::position)
            .def_readwrite("orientation", &wbc_py::Pose::orientation);

    bp::class_<wbc_py::RigidBodyStateSE3>("RigidBodyStateSE3")
            .def_readwrite("pose", &wbc_py::RigidBodyStateSE3::pose)
            .def_readwrite("twist", &wbc_py::RigidBodyStateSE3::twist)
            .def_readwrite("acceleration", &wbc_py::RigidBodyStateSE3::acceleration)
            .def_readwrite("wrench", &wbc_py::RigidBodyStateSE3::wrench);

    bp::class_<wbc_py::Twist>("Twist")
            .def_readwrite("linear", &wbc_py::Twist::linear)
            .def_readwrite("angular", &wbc_py::Twist::angular);

    bp::class_<wbc_py::Acceleration>("Acceleration")
            .def_readwrite("linear", &wbc_py::Acceleration::linear)
            .def_readwrite("angular", &wbc_py::Acceleration::angular);

    bp::class_<wbc_py::Wrench>("Wrench")
            .def_readwrite("force", &wbc_py::Wrench::force)
            .def_readwrite("torque", &wbc_py::Wrench::torque);

    bp::class_<wbc_py::RobotModelConfig>("RobotModelConfig")
            .def_readwrite("file",                 &wbc_py::RobotModelConfig::file)
            .def_readwrite("submechanism_file",    &wbc_py::RobotModelConfig::submechanism_file)
            .def_readwrite("joint_names",          &wbc_py::RobotModelConfig::joint_names)
            .def_readwrite("actuated_joint_names", &wbc_py::RobotModelConfig::actuated_joint_names)
            .def_readwrite("floating_base",        &wbc_py::RobotModelConfig::floating_base)
            .def_readwrite("world_frame_id",       &wbc_py::RobotModelConfig::world_frame_id)
            .def_readwrite("floating_base_state",  &wbc_py::RobotModelConfig::floating_base_state)
            .def_readwrite("contact_points",       &wbc_py::RobotModelConfig::contact_points);

    bp::class_<wbc_py::RobotModelHyrodyn>("RobotModelHyrodyn")
            .def("configure",&wbc_py::RobotModelHyrodyn::configure)
            .def("update",&wbc_py::RobotModelHyrodyn::update)
            .def("rigidBodyState",&wbc_py::RobotModelHyrodyn::rigidBodyState)
            .def("spaceJacobian",&wbc_py::RobotModelHyrodyn::spaceJacobian)
            .def("bodyJacobian",&wbc_py::RobotModelHyrodyn::bodyJacobian)
            .def("spatialAccelerationBias",&wbc_py::RobotModelHyrodyn::spatialAccelerationBias)
            .def("jointSpaceInertiaMatrix",&wbc_py::RobotModelHyrodyn::jointSpaceInertiaMatrix)
            .def("biasForces",&wbc_py::RobotModelHyrodyn::biasForces);

}

