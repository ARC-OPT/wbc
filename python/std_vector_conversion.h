#pragma once

#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>

namespace py = boost::python;

namespace pygen {

/*
 *  List -> std vector converters
 */

template <class StdVectorType>
struct python_list_to_std_vector {
    python_list_to_std_vector()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<StdVectorType>());
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

        using storage_type = py::converter::rvalue_from_python_storage<StdVectorType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) StdVectorType;
        StdVectorType& vec = *static_cast<StdVectorType*>(storage);

        vec.resize(len);

        for (long i = 0; i < len; ++i)
            vec[i] = py::extract<typename StdVectorType::value_type>(arr[i]);

        data->convertible = storage;
    }
};

/*
 *   To std vector -> List convertors
 */

template <typename StdVectorType>
struct std_vector_to_python_list {
    static PyObject* convert(const StdVectorType& vec)
    {
        boost::python::list l;
        for (int i = 0; i < vec.size(); i++)
            l.append(vec[i]);
        return py::incref(l.ptr());
    }
};


/*
 * Main converters
 */
template <typename StdVectorType>
void convertStdVector()
{
    python_list_to_std_vector<StdVectorType>();
    py::to_python_converter<StdVectorType, std_vector_to_python_list<StdVectorType> >();
}

} // namespace pygen
