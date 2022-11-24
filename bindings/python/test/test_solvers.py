from wbc.solvers.hls_solver import *
from wbc.solvers.qpoases_solver import *
from wbc.core import *
import nose
import numpy as np

def run(solver):
    nc = 6
    nj = 6

    hqp = HierarchicalQP()
    qp = QuadraticProgram()
    qp.resize(nj,nc,0,False)
    qp.A = [[0.642, 0.706, 0.565,  0.48,  0.59, 0.917],
            [0.553, 0.087,  0.43,  0.71, 0.148,  0.87],
            [0.249, 0.632, 0.711,  0.13, 0.426, 0.963],
            [0.682, 0.123, 0.998, 0.716, 0.961, 0.901],
            [0.315, 0.551, 0.462, 0.221, 0.638, 0.244],
            [0.891, 0.019, 0.716, 0.534, 0.725, 0.633]]
    y_ref = [0.833, 0.096, 0.078, 0.971, 0.883, 0.366]
    qp.b = y_ref
    qp.g = [0]*nj
    qp.H = np.eye(nj).tolist()
    hqp.prios = [qp]
    solver_output = solver.solve(hqp)
    y_solution = np.array(qp.A).dot(solver_output)
    assert np.all(np.isclose(y_solution - y_ref,np.zeros(nj)))

def test_hierarchial_ls_solver():
    solver = HierarchicalLSSolver()
    solver.setMaxSolverOutputNorm(100.0)
    assert solver.getMaxSolverOutputNorm() == 100.0
    solver.setMinEigenvalue(1e-7)
    assert solver.getMinEigenvalue() == 1e-7

    run(solver)

def test_qp_oases_solver():
    solver = QPOASESSolver()
    solver.setMaxNoWSR(100)
    assert solver.getMaxNoWSR() == 100

    run(solver)

    assert(solver.getNoWSR() < 100)
    assert(solver.getReturnValue() == 0)

if __name__ == '__main__':
    test_qp_oases_solver()
