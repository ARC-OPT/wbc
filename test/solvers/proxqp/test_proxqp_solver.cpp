#include <boost/test/unit_test.hpp>
#include <iostream>
#include <sys/time.h>
#include "core/QuadraticProgram.hpp"
#include "solvers/proxqp/ProxQPSolver.hpp"

using namespace wbc;
using namespace std;

BOOST_AUTO_TEST_CASE(solver_proxqp_without_constraints)
{
    srand (time(NULL));

    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 0;
    const int NO_IN_CONSTRAINTS = 0;
    const bool WITH_BOUNDS = false;
    const int NO_WSR = 20;

    // Solve the problem min(||Ax-b||) without constraints --> encode the task as part of the cost function
    // Standard form of QP is x^T*H*x + x^T*g --> Choose H = A^T*A and g = -(A^T*y)^T

    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, NO_IN_CONSTRAINTS, WITH_BOUNDS);

    // Task Jacobian
    base::Matrix6d A;
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    // Desired task space reference
    base::Vector6d y;
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;

    qp.H = A.transpose()*A;
    qp.g = -(A.transpose()*y).transpose();

    qp.check();
    wbc::HierarchicalQP hqp;
    hqp << qp;

    ProxQPSolver solver;
    solver.setMaxNIter(NO_WSR);

    BOOST_CHECK(solver.getMaxNIter() == NO_WSR);

    base::VectorXd solver_output;

    struct timeval start, end;
    gettimeofday(&start, NULL);

    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    Eigen::VectorXd test = A*solver_output;
    //cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_JOINTS; j++)
        BOOST_CHECK(fabs(test(j) - y(j)) < 1e-9);

    //cout<<"\n............................."<<endl;
}

BOOST_AUTO_TEST_CASE(solver_proxqp_with_equality_constraints)
{
    srand (time(NULL));

    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 6;
    const int NO_IN_CONSTRAINTS = 0;
    const bool WITH_BOUNDS = false;
    const int NO_WSR = 20;

    // Solve the problem min(||x||), subject Ax=b --> encode the task as constraint
    // Standard form of QP is x^T*H*x + x^T*g --> Choose H = I  and g = 0

    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, NO_IN_CONSTRAINTS, WITH_BOUNDS);

    qp.g.setZero();
    qp.H.setIdentity();
    // Task Jacobian
    base::Matrix6d A;
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    qp.A = A;
    // Desired task space reference
    base::Vector6d y;
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;
    qp.b = y;

    qp.check();
    wbc::HierarchicalQP hqp;
    hqp << qp;

    ProxQPSolver solver;
    solver.setMaxNIter(NO_WSR);

    BOOST_CHECK(solver.getMaxNIter() == NO_WSR);

    base::VectorXd solver_output;

    struct timeval start, end;
    gettimeofday(&start, NULL);
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    Eigen::VectorXd test = A*solver_output;
    //cout<<"Test: A * q_dot = "<<test.transpose();
    for(uint j = 0; j < NO_EQ_CONSTRAINTS; j++)
        BOOST_CHECK(fabs(test(j) - y(j)) < 1e-9);

    //cout<<"\n............................."<<endl;
}

BOOST_AUTO_TEST_CASE(solver_proxqp_with_inequality_constraints)
{
    srand (time(NULL));

    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 0;
    const int NO_IN_CONSTRAINTS = 6;
    const bool WITH_BOUNDS = false;
    const int NO_WSR = 40;

    // Solve the problem min(||x||), subject Ax=b --> encode the task as constraint
    // Standard form of QP is x^T*H*x + x^T*g --> Choose H = I  and g = 0

    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, NO_IN_CONSTRAINTS, WITH_BOUNDS);

    qp.g.setZero();
    qp.H.setIdentity();
    // Task Jacobian
    base::Matrix6d A;
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    qp.C = A;
    // Desired task space reference
    base::Vector6d y;
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;
    qp.lower_y = y - Eigen::VectorXd::Constant(qp.nq, 1e-1);
    qp.upper_y = y + Eigen::VectorXd::Constant(qp.nq, 1e-1);

    qp.check();
    wbc::HierarchicalQP hqp;
    hqp << qp;

    ProxQPSolver solver;
    solver.setMaxNIter(NO_WSR);

    BOOST_CHECK(solver.getMaxNIter() == NO_WSR);

    base::VectorXd solver_output;

    struct timeval start, end;
    gettimeofday(&start, NULL);
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    Eigen::VectorXd test = A*solver_output;
    
    for(uint j = 0; j < NO_JOINTS; ++j)
        BOOST_CHECK((qp.lower_y(j)-1e-9) <= test(j) && test(j) <= (qp.upper_y(j)+1e-9));
}

BOOST_AUTO_TEST_CASE(solver_proxqp_bounded)
{
    srand (time(NULL));

    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 0;
    const int NO_IN_CONSTRAINTS = 0;
    const bool WITH_BOUNDS = true;
    const int NO_WSR = 200;

    // Solve the problem min(||Ax-b||) with bound constraints --> encode the task as part of the cost function
    // Standard form of QP is x^T*H*x + x^T*g --> Choose H = A^T*A and g = -(A^T*y)^T

    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, NO_IN_CONSTRAINTS, WITH_BOUNDS);

    // Task Jacobian
    base::Matrix6d A;
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    // Desired task space reference
    base::Vector6d y;
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;

    qp.H = A.transpose()*A;
    qp.g = -(A.transpose()*y).transpose();

    qp.lower_x.setConstant(-1e10);
    qp.upper_x.setConstant(+1e10);

    qp.check();
    wbc::HierarchicalQP hqp;
    hqp << qp;

    ProxQPSolver solver;
    solver.setMaxNIter(NO_WSR);

    BOOST_CHECK(solver.getMaxNIter() == NO_WSR);

    base::VectorXd solver_output;

    struct timeval start, end;
    gettimeofday(&start, NULL);

    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    gettimeofday(&end, NULL);
    long useconds = end.tv_usec - start.tv_usec;

    for(uint j = 0; j < NO_JOINTS; ++j)
        BOOST_CHECK((qp.lower_x(j)-1e-9) <= solver_output(j) && solver_output(j) <= (qp.upper_x(j)+1e-9));

}
