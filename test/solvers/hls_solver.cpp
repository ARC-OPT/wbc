#include <boost/test/unit_test.hpp>
#include "solvers/hls/HierarchicalLSSolver.hpp"
#include "types/QuadraticProgram.hpp"
#include <iostream>
#include <sys/time.h>

using namespace wbc_solvers;
using namespace std;

BOOST_AUTO_TEST_CASE(solver_hls)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 2;
    const double NORM_MAX = 5.75;
    const double MIN_EIGENVALUE = 1e-9;

    HierarchicalLSSolver solver;
    vector<int> ny_per_prio(1,NO_CONSTRAINTS);

    BOOST_CHECK_EQUAL(solver.configure(ny_per_prio, NO_JOINTS), true);

    solver.setMaxSolverOutputNorm(NORM_MAX);
    solver.setMinEigenvalue(MIN_EIGENVALUE);

    BOOST_CHECK(solver.getMinEigenvalue() == MIN_EIGENVALUE);
    BOOST_CHECK(solver.getMaxSolverOutputNorm() == NORM_MAX);

    wbc::QuadraticProgram prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);
    prio_0.A << 0.026, 0.203, 0.451, 0.915, 0.161, 0.151;
    prio_0.lower_y << 0.49, 0.787;

    wbc::HierarchicalQP hqp;
    hqp.Wq.setOnes(NO_JOINTS);
    hqp << prio_0;

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
    cout<<"Constraint Matrix A:"<<endl; cout<<prio_0.A<<endl;
    cout<<"Reference: y = "<<prio_0.lower_y.transpose()<<endl;

    cout<<"\nSolver Output: q_dot = "<<solver_output.transpose()<<endl;
    Eigen::VectorXd test = prio_0.A*solver_output;
    cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_CONSTRAINTS; j++)
        BOOST_CHECK(fabs(test(j) - prio_0.lower_y(j)) < 1e-9);

    cout<<"\n............................."<<endl;
}
