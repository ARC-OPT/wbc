#include <boost/test/unit_test.hpp>
#include "solvers/hls/HierarchicalLSSolver.hpp"
#include "types/QuadraticProgram.hpp"
#include <iostream>
#include <sys/time.h>

using namespace wbc;
using namespace std;

BOOST_AUTO_TEST_CASE(solver_hls)
{
    // For small size problems like the one below this solver outperforms qpoases by factor of at least 2
    // Also computation time is quite constant, while for qpoases it might vary a lot, depending on the number
    // iterations

    srand (time(NULL));

    const uint NO_JOINTS = 6;
    const uint NO_CONSTRAINTS = 6;
    const double NORM_MAX = 100;
    const double MIN_EIGENVALUE = 1e-9;

    HierarchicalLSSolver solver;
    vector<int> ny_per_prio(1,NO_CONSTRAINTS);

    BOOST_CHECK_EQUAL(solver.configure(ny_per_prio, NO_JOINTS), true);

    solver.setMaxSolverOutputNorm(NORM_MAX);
    solver.setMinEigenvalue(MIN_EIGENVALUE);

    BOOST_CHECK(solver.getMinEigenvalue() == MIN_EIGENVALUE);
    BOOST_CHECK(solver.getMaxSolverOutputNorm() == NORM_MAX);

    wbc::QuadraticProgram qp;
    qp.resize(NO_CONSTRAINTS, NO_JOINTS);
    base::Vector6d y;
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;
    qp.lower_y = y;
    qp.upper_y = y;

    base::Matrix6d A;
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    qp.A = A;

    wbc::HierarchicalQP hqp;
    hqp.Wq.setOnes(NO_JOINTS);
    hqp << qp;

    base::VectorXd solver_output;
    struct timeval start, end;
    gettimeofday(&start, NULL);
    solver.solve(hqp, solver_output);
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    cout<<"\n----------------------- Test Results ----------------------"<<endl<<endl;
    std::cout<<"Solver took "<<useconds<<" us "<<std::endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;
    cout<<"No of constraints: "<<NO_CONSTRAINTS<<endl;

    cout<<"\nSolver Input:"<<endl;
    cout<<"Constraint Matrix A:"<<endl; cout<<A<<endl;
    cout<<"Reference: y = "<<y.transpose()<<endl;

    cout<<"\nSolver Output: q_dot = "<<solver_output.transpose()<<endl;
    Eigen::VectorXd test = A*solver_output;
    cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_CONSTRAINTS; j++)
        BOOST_CHECK(fabs(test(j) - y(j)) < 1e-9);

    cout<<"\n............................."<<endl;
}
