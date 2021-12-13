// MIT License

// Copyright (c) 2019 Vincent SAMY

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <Eigen/Core>
#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/numpy.hpp>

namespace py = boost::python;
namespace np = boost::python::numpy;

namespace pygen {

/*
 *     *****************         List -> Eigen converters      *****************
 */


/** Conversion from a python list to an Eigen Vector
 * This convert list of type T (int, float32, float64, ...) to an Eigen::VectorType
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <class VectorType>
struct python_list_to_eigen_vector {
    python_list_to_eigen_vector()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<VectorType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(VectorType::ColsAtCompileTime == 1 || VectorType::RowsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Only for vectors conversion

        if (!PySequence_Check(obj_ptr)
            || (VectorType::ColsAtCompileTime == 1 && VectorType::RowsAtCompileTime != Eigen::Dynamic
                   && (PySequence_Size(obj_ptr) != VectorType::RowsAtCompileTime)) // Check Fixed-size vector
            || (VectorType::RowsAtCompileTime == 1 && VectorType::ColsAtCompileTime != Eigen::Dynamic
                   && (PySequence_Size(obj_ptr) != VectorType::ColsAtCompileTime))) // Check Fixed-size row vector
            return 0;

        if (VectorType::ColsAtCompileTime == 1 && VectorType::RowsAtCompileTime != Eigen::Dynamic && (PySequence_Size(obj_ptr) != VectorType::RowsAtCompileTime))
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        for (long i = 0; i < py::len(arr); i++)
            if (!py::extract<typename VectorType::Scalar>(arr[i]).check())
                return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list arr = py::extract<py::list>(obj_ptr);
        auto len = py::len(arr);

        using storage_type = py::converter::rvalue_from_python_storage<VectorType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) VectorType;
        VectorType& vec = *static_cast<VectorType*>(storage);

        if (VectorType::RowsAtCompileTime == Eigen::Dynamic || VectorType::ColsAtCompileTime == Eigen::Dynamic)
            vec.resize(len);

        for (long i = 0; i < len; ++i)
            vec(i) = py::extract<typename VectorType::Scalar>(arr[i]);

        data->convertible = storage;
    }
};

/** Conversion from a python list to an Eigen Matrix
 * This convert list of list of type T (int, float32, float64, ...) to an Eigen::MatrixType
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <class MatrixType>
struct python_list_to_eigen_matrix {
    python_list_to_eigen_matrix()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<MatrixType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1, "Passed a Vector into a Matrix generator"); // Only for matrix conversion

        auto checkNestedList = [](const py::list& list) {
            py::extract<py::list> nested_list(list[0]);
            if (!nested_list.check())
                return false;

            auto cols = py::len(nested_list());
            for (long i = 1; i < py::len(list); ++i) {
                py::extract<py::list> nested_list(list[i]);
                if (!nested_list.check() || py::len(nested_list) != cols) // Check nested list size
                    return false;
                for (long j = 0; j < cols; ++j)
                    if (!py::extract<typename MatrixType::Scalar>(nested_list()[j]).check()) // Check list type
                        return false;
            }

            return true;
        };

        py::extract<py::list> extract_list(obj_ptr);
        py::extract<py::list> extract_nested_list(extract_list());
        if (!extract_list.check() // Check list
            || !checkNestedList(extract_list()) // Check nested lists
            || (MatrixType::RowsAtCompileTime != Eigen::Dynamic && MatrixType::RowsAtCompileTime != py::len(extract_list())) // Check rows are the same
            || (MatrixType::ColsAtCompileTime != Eigen::Dynamic && MatrixType::ColsAtCompileTime != py::len(extract_nested_list()))) // Check cols are the same
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list arr = py::extract<py::list>(obj_ptr);
        auto rows = py::len(arr);
        auto cols = py::len(py::extract<py::list>(arr[0])());

        using storage_type = py::converter::rvalue_from_python_storage<MatrixType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) MatrixType;
        MatrixType& mat = *static_cast<MatrixType*>(storage);

        // Resize matrix if (half-)dynamic-sized matrix
        if (MatrixType::RowsAtCompileTime == Eigen::Dynamic || MatrixType::ColsAtCompileTime == Eigen::Dynamic)
            mat.resize(rows, cols);

        for (long i = 0; i < rows; ++i)
            for (long j = 0; j < cols; ++j)
                mat(i, j) = py::extract<typename MatrixType::Scalar>(arr[i][j]);

        data->convertible = storage;
    }
};

/** Conversion from a python list to an Eigen Quaternion
 * This convert a list of type T (float32, float64) to an Eigen::Quaternion type
 * The template Quaternion type should be a type as e.g. Eigen::Quaternion<double, Options>
 */
template <class QuaternionType>
struct python_list_to_eigen_quaternion {
    python_list_to_eigen_quaternion()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<QuaternionType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        //static_assert(QuaternionType::ColsAtCompileTime == 1 || QuaternionType::RowsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Only for vectors conversion

        if (!PySequence_Check(obj_ptr)
            || (PySequence_Size(obj_ptr) != 4)) // Check Fixed-size vector
            return 0;

        if (PySequence_Size(obj_ptr) != 4)
            return 0;

        py::list arr = py::extract<py::list>(obj_ptr);
        for (long i = 0; i < py::len(arr); i++)
            if (!py::extract<typename QuaternionType::Scalar>(arr[i]).check())
                return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list arr = py::extract<py::list>(obj_ptr);
        auto len = py::len(arr);

        using storage_type = py::converter::rvalue_from_python_storage<QuaternionType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) QuaternionType;
        QuaternionType& vec = *static_cast<QuaternionType*>(storage);

        vec.x() = py::extract<typename QuaternionType::Scalar>(arr[0]);
        vec.y() = py::extract<typename QuaternionType::Scalar>(arr[1]);
        vec.z() = py::extract<typename QuaternionType::Scalar>(arr[2]);
        vec.w() = py::extract<typename QuaternionType::Scalar>(arr[3]);

        data->convertible = storage;
    }
};

/** Conversion from a python list to an Eigen Transform
 * This convert a list of lists of type T (float32, float64) to an Eigen::Transform type
 * The template Transform type should be a type as e.g. Eigen::Transform<double, dim, Options>
 */
template <class TransformType>
struct python_list_to_eigen_transform {
    python_list_to_eigen_transform()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<TransformType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(TransformType::Dim != 1, "Passed a Vector into a Matrix generator"); // Only for matrix conversion

        auto checkNestedList = [](const py::list& list) {
            py::extract<py::list> nested_list(list[0]);
            if (!nested_list.check())
                return false;

            auto cols = py::len(nested_list());
            for (long i = 1; i < py::len(list); ++i) {
                py::extract<py::list> nested_list(list[i]);
                if (!nested_list.check() || py::len(nested_list) != cols) // Check nested list size
                    return false;
                for (long j = 0; j < cols; ++j)
                    if (!py::extract<typename TransformType::Scalar>(nested_list()[j]).check()) // Check list type
                        return false;
            }

            return true;
        };

        py::extract<py::list> extract_list(obj_ptr);
        py::extract<py::list> extract_nested_list(extract_list());
        if (!extract_list.check() // Check list
            || !checkNestedList(extract_list()) // Check nested lists
            || (TransformType::Dim != py::len(extract_list())) // Check rows are the same
            || (TransformType::Dim != py::len(extract_nested_list()))) // Check cols are the same
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        py::list arr = py::extract<py::list>(obj_ptr);
        auto rows = py::len(arr);
        auto cols = py::len(py::extract<py::list>(arr[0])());

        using storage_type = py::converter::rvalue_from_python_storage<TransformType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) TransformType;
        TransformType& mat = *static_cast<TransformType*>(storage);

        // Resize matrix if (half-)dynamic-sized matrix
        //if (TransformType::RowsAtCompileTime == Eigen::Dynamic || TransformType::ColsAtCompileTime == Eigen::Dynamic)
        //    mat.resize(rows, cols);

        for (long i = 0; i < rows; ++i)
            for (long j = 0; j < cols; ++j)
                mat(i, j) = py::extract<typename TransformType::Scalar>(arr[i][j]);

        data->convertible = storage;
    }
};


/*
 *     *****************         Numpy -> Eigen converters      *****************
 */

/** Conversion from a numpy ndarray to an Eigen Vector
 * This convert numpy.array of dtype T (int, float32, float64, ...) to an Eigen::VectorType
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <typename VectorType>
struct numpy_array_to_eigen_vector {
    numpy_array_to_eigen_vector()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<VectorType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(VectorType::RowsAtCompileTime == 1 || VectorType::ColsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Check that in c++ side, it is an Eigen vector
        py::extract<np::ndarray> arr(obj_ptr);
        if (!arr.check() // Check it is a numpy array
            || arr().get_nd() != 1 // Check array dimension (does not allow numpy array of type (1, 3), needs to ravel it first)
            || arr().get_dtype() != np::dtype::get_builtin<typename VectorType::Scalar>() // Check type
            || (VectorType::RowsAtCompileTime == 1
                   && VectorType::ColsAtCompileTime != Eigen::Dynamic
                   && VectorType::ColsAtCompileTime != arr().shape(0)) // Check vector size in case of fixed-size array (for a row-vector)
            || (VectorType::ColsAtCompileTime == 1
                   && VectorType::RowsAtCompileTime != Eigen::Dynamic
                   && VectorType::RowsAtCompileTime != arr().shape(0))) // Check vector size in case of fixed-size array (for a column-vector)
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

        using storage_type = py::converter::rvalue_from_python_storage<VectorType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) VectorType;
        VectorType& vec = *static_cast<VectorType*>(storage);
        // Resize for dynamic-sized matrices
        if (VectorType::RowsAtCompileTime == Eigen::Dynamic || VectorType::ColsAtCompileTime == Eigen::Dynamic)
            vec.resize(arr.shape(0));

        // Extract values. The type has been check in the convertible function
        for (int i = 0; i < arr.shape(0); ++i)
            vec(i) = py::extract<typename VectorType::Scalar>(arr[i]);

        data->convertible = storage;
    }
};

/** Conversion from a numpy ndarray to an Eigen Matrix
 * This convert numpy.array of dtype T (int, float32, float64, ...) to an Eigen::MatrixType
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <typename MatrixType>
struct numpy_array_to_eigen_matrix {
    numpy_array_to_eigen_matrix()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<MatrixType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1, "Passed a Vector into a Matrix generator"); // Only for matrix conversion

        py::extract<np::ndarray> arr(obj_ptr);
        if (!arr.check() // Check it is a numpy array
            || arr().get_nd() != 2 // Check array dimension
            || arr().get_dtype() != np::dtype::get_builtin<typename MatrixType::Scalar>() // Check type
            || (MatrixType::RowsAtCompileTime != Eigen::Dynamic && MatrixType::RowsAtCompileTime != arr().shape(0)) // Check rows are the same
            || (MatrixType::ColsAtCompileTime != Eigen::Dynamic && MatrixType::ColsAtCompileTime != arr().shape(1))) // Check cols are the same
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

        using storage_type = py::converter::rvalue_from_python_storage<MatrixType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) MatrixType;
        MatrixType& mat = *static_cast<MatrixType*>(storage);
        // Resize for dynamic-sized matrices
        // For half dynamic-sized matrices, the fixed-size part has been check in the convertible function
        if (MatrixType::RowsAtCompileTime == Eigen::Dynamic || MatrixType::ColsAtCompileTime == Eigen::Dynamic)
            mat.resize(arr.shape(0), arr.shape(1));

        // Extract values. The type has been check in the convertible function
        for (int i = 0; i < arr.shape(0); ++i)
            for (int j = 0; j < arr.shape(1); ++j)
                mat(i, j) = py::extract<typename MatrixType::Scalar>(arr[i][j]);

        data->convertible = storage;
    }
};

template <typename QuaternionType>
struct numpy_array_to_eigen_quaternion{
    numpy_array_to_eigen_quaternion()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<QuaternionType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        //static_assert(QuaternionType::RowsAtCompileTime == 1 || QuaternionType::ColsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Check that in c++ side, it is an Eigen vector
        py::extract<np::ndarray> arr(obj_ptr);
        if (!arr.check() // Check it is a numpy array
            || arr().get_nd() != 1 // Check array dimension (does not allow numpy array of type (1, 3), needs to ravel it first)
            || arr().get_dtype() != np::dtype::get_builtin<typename QuaternionType::Scalar>() // Check type
            || 4 != arr().shape(0) // Check vector size in case of fixed-size array (for a row-vector)
            || 4 != arr().shape(0)) // Check vector size in case of fixed-size array (for a column-vector)
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

        using storage_type = py::converter::rvalue_from_python_storage<QuaternionType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) QuaternionType;
        QuaternionType& vec = *static_cast<QuaternionType*>(storage);
        // Resize for dynamic-sized matrices
        //if (QuaternionType::RowsAtCompileTime == Eigen::Dynamic || QuaternionType::ColsAtCompileTime == Eigen::Dynamic)
        //    vec.resize(arr.shape(0));

        // Extract values. The type has been check in the convertible function
        vec.x() = py::extract<typename QuaternionType::Scalar>(arr[0]);
        vec.y() = py::extract<typename QuaternionType::Scalar>(arr[1]);
        vec.z() = py::extract<typename QuaternionType::Scalar>(arr[2]);
        vec.w() = py::extract<typename QuaternionType::Scalar>(arr[3]);

        data->convertible = storage;
    }
};

template <typename TransformType>
struct numpy_array_to_eigen_transform {
    numpy_array_to_eigen_transform()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<TransformType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        static_assert(TransformType::Dim != 1, "Passed a Vector into a Matrix generator"); // Only for matrix conversion

        py::extract<np::ndarray> arr(obj_ptr);
        if (!arr.check() // Check it is a numpy array
            || arr().get_nd() != 2 // Check array dimension
            || arr().get_dtype() != np::dtype::get_builtin<typename TransformType::Scalar>() // Check type
            || TransformType::Dim != arr().shape(0) // Check rows are the same
            || TransformType::Dim != arr().shape(1)) // Check cols are the same
            return 0;

        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, py::converter::rvalue_from_python_stage1_data* data)
    {
        np::ndarray arr = py::extract<np::ndarray>(obj_ptr);

        using storage_type = py::converter::rvalue_from_python_storage<TransformType>;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

        new (storage) TransformType;
        TransformType& mat = *static_cast<TransformType*>(storage);
        // Resize for dynamic-sized matrices
        // For half dynamic-sized matrices, the fixed-size part has been check in the convertible function
        //if (TransformType::Dim == Eigen::Dynamic || TransformType::Dim == Eigen::Dynamic)
        //    mat.resize(arr.shape(0), arr.shape(1));

        // Extract values. The type has been check in the convertible function
        for (int i = 0; i < arr.shape(0); ++i)
            for (int j = 0; j < arr.shape(1); ++j)
                mat(i, j) = py::extract<typename TransformType::Scalar>(arr[i][j]);

        data->convertible = storage;
    }
};

/*
 *       ******************       Eigen -> Numpy converters       ******************
 */

/** Conversion from an Eigen Vector to an numpy ndarray
 * This convert Eigen::VectorType of type T (int, float, double, ...) to an numpy.array
 * The template VectorType should be a type as Eigen::Matrix<T, rows, 1> or Eigen::Matrix<T, 1, cols>
 * Note it should also work for Eigen::Array
 */
template <typename VectorType>
struct eigen_vector_to_numpy_array {
    static PyObject* convert(const VectorType& mat)
    {
        static_assert(VectorType::ColsAtCompileTime == 1 || VectorType::RowsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Ensure that it is a vector

        np::dtype dt = np::dtype::get_builtin<typename VectorType::Scalar>();
        auto shape = py::make_tuple(mat.size());
        np::ndarray mOut = np::empty(shape, dt);

        for (Eigen::Index i = 0; i < mat.size(); ++i)
            mOut[i] = mat(i);

        return py::incref(mOut.ptr());
    }
};

/** Conversion from an Eigen Matrix to an numpy ndarray
 * This convert Eigen::MatrixType of type T (int, float, double, ...) to an numpy.array
 * The template MatrixType should be a type as Eigen::Matrix<T, rows, cols>
 * Note it should also work for Eigen::Array
 */
template <typename MatrixType>
struct eigen_matrix_to_numpy_array {
    static PyObject* convert(const MatrixType& mat)
    {
        static_assert(MatrixType::ColsAtCompileTime != 1 && MatrixType::RowsAtCompileTime != 1, "Passed a Vector into a Matrix generator"); // Ensure that it is not a vector

        np::dtype dt = np::dtype::get_builtin<typename MatrixType::Scalar>();
        auto shape = py::make_tuple(mat.rows(), mat.cols());
        np::ndarray mOut = np::empty(shape, dt);

        for (Eigen::Index i = 0; i < mat.rows(); ++i)
            for (Eigen::Index j = 0; j < mat.cols(); ++j)
                mOut[i][j] = mat(i, j);

        return py::incref(mOut.ptr());
    }
};

/** Conversion from an Eigen Quaternion to an numpy ndarray
 * This converts Eigen::Quaternion of type T (float, double) to an numpy.array.
 * Order is [im_x,im_y,im_z,re]!
 * The template QauternionType should be a type as Eigen::Quaternion<double, Options>
 */
template <typename QuaternionType>
struct eigen_quaternion_to_numpy_array {
    static PyObject* convert(const QuaternionType& mat)
    {
        //static_assert(VectorType::ColsAtCompileTime == 1 || VectorType::RowsAtCompileTime == 1, "Passed a Matrix into a Vector generator"); // Ensure that it is a vector

        np::dtype dt = np::dtype::get_builtin<typename QuaternionType::Scalar>();
        auto shape = py::make_tuple(4);
        np::ndarray mOut = np::empty(shape, dt);

        for (Eigen::Index i = 0; i < 4; ++i)
            mOut[i] = mat.coeffs()[i];

        return py::incref(mOut.ptr());
    }
};

/** Conversion from an Eigen Transform to an numpy matrix
 * This converts Eigen Transform of type T (float, double) to an numpy.matrix.
 * The template TransformType should be a type as Eigen::Transform<double, dim, Options>
 */
template <typename TransformType>
struct eigen_transform_to_numpy_array {
    static PyObject* convert(const TransformType& mat)
    {
        static_assert(TransformType::Dim != 1, "Passed a Vector into a Matrix generator"); // Ensure that it is not a vector

        np::dtype dt = np::dtype::get_builtin<typename TransformType::Scalar>();
        auto shape = py::make_tuple(mat.rows(), mat.cols());
        np::ndarray mOut = np::empty(shape, dt);

        for (Eigen::Index i = 0; i < mat.rows(); ++i)
            for (Eigen::Index j = 0; j < mat.cols(); ++j)
                mOut[i][j] = mat(i, j);

        return py::incref(mOut.ptr());
    }
};

/*
 * **************************************** Main converters  *******************************************
 */

/** Eigen Matrix and Array conversion
 * Generate the conversion for Eigen MatrixType (MatrixXd, Array2Xf, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen MatrixType
 */
template <typename MatrixType>
void convertMatrix(bool isListConvertible = true)
{
    // python -> eigen
    numpy_array_to_eigen_matrix<MatrixType>();

    // list -> eigen
    if (isListConvertible)
        python_list_to_eigen_matrix<MatrixType>();

    // eigen -> python
    py::to_python_converter<MatrixType, eigen_matrix_to_numpy_array<MatrixType> >(); // Vector2<T>
}

/** Eigen quaternion conversion
 * Generate the conversion for Eigen quaternion type (Quaterniond, Quaternionf)
 * \param isListConvertible if true, generate conversion from python list to Eigen quaternion type
 */
template <typename QuaternionType>
void convertQuaternion(bool isListConvertible = true)
{
    // python -> eigen
    numpy_array_to_eigen_quaternion<QuaternionType>();

    // list -> eigen
    if (isListConvertible)
        python_list_to_eigen_quaternion<QuaternionType>();

    // eigen -> python
    py::to_python_converter<QuaternionType, eigen_quaternion_to_numpy_array<QuaternionType> >();
}

/** Eigen Transform conversion
 * Generate the conversion for Eigen Transform (Affine2d, Affine3d,  ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen Transform
 */
template <typename TransformType>
void convertTransform(bool isListConvertible = true)
{
    // python -> eigen
    numpy_array_to_eigen_transform<TransformType>();

    // list -> eigen
    if (isListConvertible)
        python_list_to_eigen_transform<TransformType>();

    // eigen -> python
    py::to_python_converter<TransformType, eigen_transform_to_numpy_array<TransformType> >(); // Vector2<T>
}

/** Eigen Vector and Array conversion
 * Generate the conversion for Eigen VectorType (VectorXd, RowVector4f, ArrayXf, ...)
 * \param isListConvertible if true, generate conversion from python list to Eigen VectorType
 */
template <typename VectorType>
void convertVector(bool isListConvertible = true)
{
    // python -> eigen
    numpy_array_to_eigen_vector<VectorType>();

    // list -> eigen
    if (isListConvertible)
        python_list_to_eigen_vector<VectorType>();

    // eigen -> python
    py::to_python_converter<VectorType, eigen_vector_to_numpy_array<VectorType> >();
}

} // namespace pygen
