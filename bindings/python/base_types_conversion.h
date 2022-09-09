#pragma once

#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/numpy.hpp>
#include <base/JointState.hpp>
#include <base/samples/Joints.hpp>
#include <base/JointLimits.hpp>

namespace py = boost::python;
namespace np = boost::python::numpy;

namespace wbc_py{

base::samples::Joints tobaseSamplesJoints(const base::NamedVector<base::JointState> &joints_in){
    base::samples::Joints joints_out;
    joints_out.elements = joints_in.elements;
    joints_out.names = joints_in.names;
    joints_out.time = base::Time::now();
    return joints_out;
}

base::NamedVector<base::JointState> toNamedVector(const base::samples::Joints &joints_in){
    base::NamedVector<base::JointState> joints_out;
    joints_out.elements = joints_in.elements;
    joints_out.names = joints_in.names;
    return joints_out;
}

base::NamedVector<base::JointLimitRange> fromJointLimits(const base::JointLimits &limits_in){
    base::NamedVector<base::JointLimitRange> limits_out;
    limits_out.elements = limits_in.elements;
    limits_out.names = limits_in.names;
    return limits_out;
}


}

namespace pygen {

template <class JointVectorType>
struct python_list_to_joint_vector {
    python_list_to_joint_vector()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<JointVectorType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PySequence_Check(obj_ptr))
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list arr = py::extract<py::list>(obj_ptr);
        auto len = py::len(arr);

        using storage_type = py::converter::rvalue_from_python_storage<JointVectorType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) JointVectorType;
        JointVectorType& vec = *static_cast<JointVectorType*>(storage);

        vec.resize(len);

        for (long i = 0; i < len; ++i)
            vec[i] = py::extract<base::JointState>(arr[i]);

        data->convertible = storage;
    }
};

} // namespace pygen
