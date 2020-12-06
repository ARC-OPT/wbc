#include <boost/test/unit_test.hpp>
#include <iostream>
#include <sys/time.h>
#include "types/QuadraticProgram.hpp"
#include "solvers/qpoases/QPOasesSolver.hpp"

using namespace wbc_solvers;
using namespace std;
using namespace qpOASES;

BOOST_AUTO_TEST_CASE(solver_qp_oases)
{
    srand (time(NULL));

    const int NO_JOINTS = 6;
    const int NO_CONSTRAINTS = 6;
    const int NO_WSR = 20;

    // Simple problem:
    //  - Hessian is identity
    //  - Joint variables are unbounded (ub == lb)
    //  - Only equality constraints (lbA == ubA)
    //  - Gradient vector is zero
    //  - Constraint matrix is random
    wbc::QuadraticProgram qp;
    qp.resize(NO_CONSTRAINTS, NO_JOINTS);
    qp.H.setIdentity();
    qp.g.setZero();
    qp.lower_x.resize(0);
    qp.upper_x.resize(0);
    qp.A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
            0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
            0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
            0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
            0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
            0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    qp.lower_y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;
    qp.upper_y = qp.lower_y;
    wbc::HierarchicalQP hqp;
    hqp << qp;

    QPOASESSolver solver;
    solver.setMaxNoWSR(NO_WSR);

    BOOST_CHECK(solver.getMaxNoWSR() == NO_WSR);

    base::VectorXd solver_output;

    struct timeval start, end;
    gettimeofday(&start, NULL);
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    /*cout<<"\n----------------------- Test Results ----------------------"<<endl<<endl;
    std::cout<<"Solver took "<<useconds<<" us "<<std::endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;
    cout<<"No of constraints: "<<NO_CONSTRAINTS<<endl;

    cout<<"\nSolver Input:"<<endl;
    cout<<"Constraint Matrix A:"<<endl; cout<<qp.A<<endl;
    cout<<"Reference: y = "<<qp.lower_y.transpose()<<endl;

    cout<<"\nSolver Output: q_dot = "<<solver_output.transpose()<<endl;*/
    Eigen::VectorXd test = qp.A*solver_output;
    //cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_CONSTRAINTS; j++)
        BOOST_CHECK(fabs(test(j) - qp.lower_y(j)) < 1e-9);

    //cout<<"\n............................."<<endl;
}
