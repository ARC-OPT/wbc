#include "../../eigen_conversion.h"
#include "../../base_types_conversion.h"
#include "../../std_vector_conversion.h"
#include "solvers/hls/HierarchicalLSSolver.hpp"
#include "HierarchicalLSSolver.hpp"
#include "core/QuadraticProgram.hpp"

namespace wbc_py {
base::VectorXd HierarchicalLSSolver::solve(const wbc::HierarchicalQP &hqp){
    base::VectorXd solver_output;
    wbc::HierarchicalLSSolver::solve(hqp, solver_output);
    return solver_output;
}
}

BOOST_PYTHON_MODULE(hls_solver){

    np::initialize();

    py::class_<wbc_py::HierarchicalLSSolver>("HierarchicalLSSolver")
            .def("solve", &wbc_py::HierarchicalLSSolver::solve)
            .def("setMaxSolverOutputNorm", &wbc_py::HierarchicalLSSolver::setMaxSolverOutputNorm)
            .def("getMaxSolverOutputNorm", &wbc_py::HierarchicalLSSolver::getMaxSolverOutputNorm)
            .def("setMinEigenvalue", &wbc_py::HierarchicalLSSolver::setMinEigenvalue)
            .def("getMinEigenvalue", &wbc_py::HierarchicalLSSolver::getMinEigenvalue);
}


