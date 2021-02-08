#include "wbc_py.hpp"
#include <boost/python.hpp>

namespace bp = boost::python;

template<class T>
inline
bp::list to_python(const std::vector<T>& v)
{
    bp::object get_iter = bp::iterator<std::vector<T> >();
    bp::object iter = get_iter(v);
    bp::list l(iter);
    return l;
}

template< typename T >
inline
std::vector< T > from_python( const bp::list& iterable )
{
    return std::vector< T >( bp::stl_input_iterator< T >( iterable ),
                             bp::stl_input_iterator< T >( ) );
}

base::samples::RigidBodyStateSE3 from_python(wbc_py::RigidBodyStateSE3 in){
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
    out.time = base::Time::now();
    return out;
}

wbc_py::RigidBodyStateSE3 to_python(base::samples::RigidBodyStateSE3 in){
    wbc_py::RigidBodyStateSE3 out;
    for(int i = 0; i < 4; i++)
        out.pose.orientation[i] = in.pose.orientation.coeffs()[i];
    for(int i = 0; i < 3; i++){
        out.pose.position[i] = in.pose.position[i];
        out.twist.linear[i] = in.twist.linear[i];
        out.twist.angular[i] = in.twist.angular[i];
        out.acceleration.linear[i] = in.acceleration.linear[i];
        out.acceleration.angular[i] = in.acceleration.angular[i];
        out.wrench.force[i] = in.wrench.force[i];
        out.wrench.torque[i] = in.wrench.torque[i];
    }
    return out;
}

namespace  wbc_py {

bool RobotModel::configure(const RobotModelConfig &cfg){
    wbc::RobotModelConfig wbc_cfg(cfg.file,
                                  from_python<std::string>(cfg.joint_names),
                                  from_python<std::string>(cfg.actuated_joint_names),
                                  cfg.floating_base,
                                  cfg.world_frame_id,
                                  from_python(cfg.floating_base_state),
                                  from_python<std::string>(cfg.contact_points),
                                  cfg.submechanism_file);
    return robot_model->configure(wbc_cfg);
}

void RobotModel::update(bp::list names,
                        bp::list positions,
                        bp::list velocities,
                        bp::list accelerations){
    base::samples::Joints joints;
    joints.names = from_python<std::string>(names);
    for(int i = 0; i < bp::len(names); i++){
        base::JointState state;
        state.position     = bp::extract<double>(positions[i]);
        state.speed        = bp::extract<double>(velocities[i]);
        state.acceleration = bp::extract<double>(accelerations[i]);
        joints.elements.push_back(state);
    }
    joints.time = base::Time::now();
    robot_model->update(joints);
}

void RobotModel::update2(bp::list names,
                         bp::list positions,
                         bp::list velocities,
                         bp::list accelerations,
                         RigidBodyStateSE3 floating_base_state){
    base::samples::Joints joints;
    joints.names = from_python<std::string>(names);
    for(int i = 0; i < bp::len(names); i++){
        base::JointState state;
        state.position     = bp::extract<double>(positions[i]);
        state.speed        = bp::extract<double>(velocities[i]);
        state.acceleration = bp::extract<double>(accelerations[i]);
        joints.elements.push_back(state);
    }
    joints.time = base::Time::now();
    robot_model->update(joints, from_python(floating_base_state));
}

RigidBodyStateSE3 RobotModel::rigidBodyState(const std::string &root, const std::string &tip){
    return to_python(robot_model->rigidBodyState(root,tip));
}

bp::list RobotModel::spaceJacobian(const std::string& root, const std::string &tip){

}

bp::list RobotModel::bodyJacobian(const std::string& root, const std::string &tip){

}

Acceleration RobotModel::spatialAccelerationBias(const std::string& root, const std::string &tip){

}

bp::list RobotModel::jointSpaceInertiaMatrix(){

}

bp::list RobotModel::biasForces(){

}

bp::list RobotModel::jointNames(){
    return to_python(robot_model->jointNames());
}

bp::list RobotModel::actuatedJointNames(){
    return to_python(robot_model->actuatedJointNames());
}

} // namespace wbc_py

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
            .def("update",&wbc_py::RobotModelHyrodyn::update2)
            .def("rigidBodyState",&wbc_py::RobotModelHyrodyn::rigidBodyState)
            .def("spaceJacobian",&wbc_py::RobotModelHyrodyn::spaceJacobian)
            .def("bodyJacobian",&wbc_py::RobotModelHyrodyn::bodyJacobian)
            .def("spatialAccelerationBias",&wbc_py::RobotModelHyrodyn::spatialAccelerationBias)
            .def("jointSpaceInertiaMatrix",&wbc_py::RobotModelHyrodyn::jointSpaceInertiaMatrix)
            .def("biasForces",&wbc_py::RobotModelHyrodyn::biasForces)
            .def("noOfJoints",&wbc_py::RobotModelHyrodyn::noOfJoints);

    bp::class_<wbc_py::RobotModelKDL>("RobotModelKDL")
            .def("configure",&wbc_py::RobotModelKDL::configure)
            .def("update",&wbc_py::RobotModelKDL::update)
            .def("update",&wbc_py::RobotModelKDL::update2)
            .def("rigidBodyState",&wbc_py::RobotModelKDL::rigidBodyState)
            .def("spaceJacobian",&wbc_py::RobotModelKDL::spaceJacobian)
            .def("bodyJacobian",&wbc_py::RobotModelKDL::bodyJacobian)
            .def("spatialAccelerationBias",&wbc_py::RobotModelKDL::spatialAccelerationBias)
            .def("jointSpaceInertiaMatrix",&wbc_py::RobotModelKDL::jointSpaceInertiaMatrix)
            .def("biasForces",&wbc_py::RobotModelKDL::biasForces)
            .def("noOfJoints",&wbc_py::RobotModelKDL::noOfJoints);
}

