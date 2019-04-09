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

    //BOOST_CHECK_EQUAL(solver.configure(ny_per_prio, NO_JOINTS), true);

    solver.setMaxSolverOutputNorm(NORM_MAX);
    solver.setMinEigenvalue(MIN_EIGENVALUE);

    wbc::HierarchicalQP hqp;
    wbc::QuadraticProgram prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        prio_0.A.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.lower_y.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.Wy.data()[i] = (rand()%1000)/1000.0;

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
