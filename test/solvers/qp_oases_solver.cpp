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
    for(int i = 0; i < NO_CONSTRAINTS; i++){
        qp.lower_y(i) = qp.upper_y(i) = (rand()%1000)/1000.0;
        for(int j = 0; j < NO_JOINTS; j++){
            qp.A(i,j) = (rand()%1000)/1000.0;
        }
    }
    wbc::HierarchicalQP hqp;
    hqp << qp;

    QPOASESSolver solver;
    solver.setNoWSR(20);
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
    cout<<"Constraint Matrix A:"<<endl; cout<<qp.A<<endl;
    cout<<"Reference: y = "<<qp.lower_y.transpose()<<endl;

    cout<<"\nSolver Output: q_dot = "<<solver_output.transpose()<<endl;
    Eigen::VectorXd test = qp.A*solver_output;
    cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_CONSTRAINTS; j++)
        BOOST_CHECK(fabs(test(j) - qp.lower_y(j)) < 1e-9);

    cout<<"\n............................."<<endl;

    int n=0;
    while(n++ < 100){

        gettimeofday(&start, NULL);
        solver.solve(hqp, solver_output);
        gettimeofday(&end, NULL);
        useconds = end.tv_usec - start.tv_usec;

        cout<<"\n----------------------- Test Results ----------------------"<<endl<<endl;
        std::cout<<"Solver took "<<useconds<<" us "<<std::endl;
        cout<<"No of joints: "<<NO_JOINTS<<endl;
        cout<<"No of constraints: "<<NO_CONSTRAINTS<<endl;

        cout<<"\nSolver Input:"<<endl;
        cout<<"Constraint Matrix A:"<<endl; cout<<qp.A<<endl;
        cout<<"Reference: y = "<<qp.lower_y.transpose()<<endl;

        cout<<"\nSolver Output: q_dot = "<<solver_output.transpose()<<endl;
        test = qp.A*solver_output;
        cout<<"Test: A * q_dot = "<<test.transpose();
        for(uint j = 0; j < NO_CONSTRAINTS; j++)
            BOOST_CHECK(fabs(test(j) - qp.lower_y(j)) < 1e-9);

        cout<<"\n............................."<<endl;
    }
}
