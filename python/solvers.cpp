#include "eigen_conversion.h"
#include "base_types_conversion.h"
#include "std_vector_conversion.h"
#include "solvers/hls/HierarchicalLSSolver.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"
#include "solvers.hpp"
#include "core/QuadraticProgram.hpp"

namespace wbc_py {
base::VectorXd HierarchicalLSSolver::solve(const wbc::HierarchicalQP &hqp){
    base::VectorXd solver_output;
    wbc::HierarchicalLSSolver::solve(hqp, solver_output);
    return solver_output;
}
base::VectorXd QPOASESSolver::solve(const wbc::HierarchicalQP &hqp){
    base::VectorXd solver_output;
    wbc::QPOASESSolver::solve(hqp, solver_output);
    return solver_output;
}
int QPOASESSolver::getReturnValueAsInt(){
    return (int)wbc::QPOASESSolver::getReturnValue();
}
}

BOOST_PYTHON_MODULE(solvers){

    np::initialize();

    py::class_<wbc_py::HierarchicalLSSolver>("HierarchicalLSSolver")
            .def("solve", &wbc_py::HierarchicalLSSolver::solve)
            .def("setMaxSolverOutputNorm", &wbc_py::HierarchicalLSSolver::setMaxSolverOutputNorm)
            .def("getMaxSolverOutputNorm", &wbc_py::HierarchicalLSSolver::getMaxSolverOutputNorm)
            .def("setMinEigenvalue", &wbc_py::HierarchicalLSSolver::setMinEigenvalue)
            .def("getMinEigenvalue", &wbc_py::HierarchicalLSSolver::getMinEigenvalue);
    py::class_<wbc_py::QPOASESSolver>("QPOASESSolver")
            .def("solve", &wbc_py::QPOASESSolver::solve)
            .def("setMaxNoWSR", &wbc_py::QPOASESSolver::setMaxNoWSR)
            .def("getMaxNoWSR", &wbc_py::QPOASESSolver::getMaxNoWSR)
            .def("getReturnValue", &wbc_py::QPOASESSolver::getReturnValueAsInt)
            .def("getNoWSR", &wbc_py::QPOASESSolver::getNoWSR)
            .def("getOptions", &wbc_py::QPOASESSolver::getOptions)
            .def("setOptions", &wbc_py::QPOASESSolver::setOptions);
}


