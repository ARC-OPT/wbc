#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <memory>
#include "../../../core/QuadraticProgram.hpp"
#include "../DaqpSolver.hpp"

using namespace wbc;
using namespace std;

namespace {

Eigen::MatrixXd taskJacobian(){
    Eigen::MatrixXd A(6,6);
    A << 0.642, 0.706, 0.565,  0.48,  0.59, 0.917,
         0.553, 0.087,  0.43,  0.71, 0.148,  0.87,
         0.249, 0.632, 0.711,  0.13, 0.426, 0.963,
         0.682, 0.123, 0.998, 0.716, 0.961, 0.901,
         0.891, 0.019, 0.716, 0.534, 0.725, 0.633,
         0.315, 0.551, 0.462, 0.221, 0.638, 0.244;
    return A;
}

Eigen::VectorXd taskReference(){
    Eigen::VectorXd y(6);
    y << 0.833, 0.096, 0.078, 0.971, 0.883, 0.366;
    return y;
}

}

BOOST_AUTO_TEST_CASE(solver_daqp_without_constraints)
{
    const int NO_JOINTS = 6;

    // Solve the problem min(||Ax-b||) without constraints --> encode the task as part of the cost function
    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, 0, 0, false);

    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();
    qp.H = A.transpose()*A;
    qp.g = -(A.transpose()*y);

    BOOST_CHECK(qp.isValid());
    wbc::HierarchicalQP hqp;
    hqp << qp;

    DaqpSolver solver;
    solver.setMaxNIter(100);
    BOOST_CHECK(solver.getMaxNIter() == 100);

    Eigen::VectorXd solver_output;
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
    BOOST_CHECK(solver.getExitFlag() > 0);

    Eigen::VectorXd test = A*solver_output;
    for(uint j = 0; j < NO_JOINTS; j++)
        BOOST_CHECK(fabs(test(j) - y(j)) < 1e-6);
}

BOOST_AUTO_TEST_CASE(solver_daqp_with_equality_constraints)
{
    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 6;

    // Solve the problem min(||x||), subject Ax=b --> encode the task as constraint
    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, 0, false);

    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();
    qp.g.setZero();
    qp.H.setIdentity();
    qp.A = A;
    qp.b = y;

    BOOST_CHECK(qp.isValid());
    wbc::HierarchicalQP hqp;
    hqp << qp;

    DaqpSolver solver;
    Eigen::VectorXd solver_output;
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));

    Eigen::VectorXd test = A*solver_output;
    for(uint j = 0; j < y.size(); j++)
        BOOST_CHECK(fabs(test(j) - y(j)) < 1e-6);
}

BOOST_AUTO_TEST_CASE(solver_daqp_with_inequality_constraints)
{
    const int NO_JOINTS = 6;
    const int NO_IN_CONSTRAINTS = 6;

    // Solve the problem min(||x||), subject y <= Ax <= y + 1e-4
    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, 0, NO_IN_CONSTRAINTS, false);

    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();
    qp.g.setZero();
    qp.H.setIdentity();
    qp.C = A;
    qp.lower_y = y;
    qp.upper_y = Eigen::VectorXd::Constant(NO_IN_CONSTRAINTS, 0.0001) + y;

    BOOST_CHECK(qp.isValid());
    wbc::HierarchicalQP hqp;
    hqp << qp;

    DaqpSolver solver;
    Eigen::VectorXd solver_output;
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));

    Eigen::VectorXd test = A*solver_output;
    for(uint j = 0; j < NO_IN_CONSTRAINTS; j++){
        BOOST_CHECK(test(j) >= qp.lower_y(j) - 1e-6);
        BOOST_CHECK(test(j) <= qp.upper_y(j) + 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(solver_daqp_bounded)
{
    const int NO_JOINTS = 6;

    // Solve the problem min(||Ax-b||) subject to -0.4 <= x <= 0.4
    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, 0, 0, true);

    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();
    qp.H = A.transpose()*A;
    qp.g = -(A.transpose()*y);
    qp.lower_x.setConstant(-0.4);
    qp.upper_x.setConstant(+0.4);

    BOOST_CHECK(qp.isValid());
    wbc::HierarchicalQP hqp;
    hqp << qp;

    DaqpSolver solver;
    Eigen::VectorXd solver_output;
    BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));

    for(uint j = 0; j < NO_JOINTS; ++j)
        BOOST_CHECK((qp.lower_x(j)-1e-9) <= solver_output(j) && solver_output(j) <= (qp.upper_x(j)+1e-9));
    // The unconstrained solution violates the bounds, so at least one bound must be active
    BOOST_CHECK(solver_output.cwiseAbs().maxCoeff() > 0.4 - 1e-6);
}

BOOST_AUTO_TEST_CASE(solver_daqp_mixed_constraints_and_warm_start)
{
    const int NO_JOINTS = 6;
    const int NO_EQ_CONSTRAINTS = 2;
    const int NO_IN_CONSTRAINTS = 3;

    // min ||Ax-y||, s.t. two equality constraints, three inequality constraints (one-sided), bounds
    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();

    wbc::QuadraticProgram qp;
    qp.resize(NO_JOINTS, NO_EQ_CONSTRAINTS, NO_IN_CONSTRAINTS, true);
    qp.H = A.transpose()*A + 1e-6*Eigen::MatrixXd::Identity(NO_JOINTS,NO_JOINTS);
    qp.g = -(A.transpose()*y);
    qp.A = A.topRows(NO_EQ_CONSTRAINTS);
    qp.b = 0.5*y.head(NO_EQ_CONSTRAINTS);
    qp.C = A.bottomRows(NO_IN_CONSTRAINTS);
    qp.lower_y.setConstant(-std::numeric_limits<double>::infinity());
    qp.upper_y = 0.3*y.tail(NO_IN_CONSTRAINTS);
    qp.lower_x.setConstant(-1.0);
    qp.upper_x.setConstant(+1.0);
    BOOST_CHECK(qp.isValid());

    DaqpSolver warm_solver;
    for(int k = 0; k < 20; k++){
        // Slowly varying problem data, as in a control loop
        qp.g = -(A.transpose()*(y*(1.0+0.05*k)));
        qp.b = 0.5*y.head(NO_EQ_CONSTRAINTS)*(1.0 - 0.02*k);
        wbc::HierarchicalQP hqp;
        hqp << qp;

        Eigen::VectorXd x_warm, x_cold;
        BOOST_CHECK_NO_THROW(warm_solver.solve(hqp, x_warm, true));

        DaqpSolver cold_solver;
        BOOST_CHECK_NO_THROW(cold_solver.solve(hqp, x_cold, false));

        BOOST_CHECK((x_warm-x_cold).norm() < 1e-6);
        BOOST_CHECK((qp.A*x_warm - qp.b).norm() < 1e-6);
        BOOST_CHECK(((qp.C*x_warm) - qp.upper_y).maxCoeff() < 1e-6);
        BOOST_CHECK((x_warm - qp.upper_x).maxCoeff() < 1e-6);
        BOOST_CHECK((qp.lower_x - x_warm).maxCoeff() < 1e-6);
        BOOST_CHECK(warm_solver.getDualSolution().size() == NO_JOINTS + NO_IN_CONSTRAINTS + NO_EQ_CONSTRAINTS);
    }
}

BOOST_AUTO_TEST_CASE(solver_daqp_dimension_change)
{
    Eigen::MatrixXd A = taskJacobian();
    Eigen::VectorXd y = taskReference();
    DaqpSolver solver;
    Eigen::VectorXd solver_output;

    for(int neq = 1; neq <= 6; neq++){
        wbc::QuadraticProgram qp;
        qp.resize(6, neq, 0, false);
        qp.H.setIdentity();
        qp.g.setZero();
        qp.A = A.topRows(neq);
        qp.b = y.head(neq);
        wbc::HierarchicalQP hqp;
        hqp << qp;
        BOOST_CHECK_NO_THROW(solver.solve(hqp, solver_output));
        BOOST_CHECK((qp.A*solver_output - qp.b).norm() < 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(solver_daqp_infeasible)
{
    // x0 >= 1 and x0 <= -1 is infeasible
    wbc::QuadraticProgram qp;
    qp.resize(2, 0, 2, false);
    qp.H.setIdentity();
    qp.g.setZero();
    qp.C << 1, 0,
            1, 0;
    qp.lower_y << 1, -1e30;
    qp.upper_y << 1e30, -1;
    wbc::HierarchicalQP hqp;
    hqp << qp;

    DaqpSolver solver;
    Eigen::VectorXd solver_output;
    BOOST_CHECK_THROW(solver.solve(hqp, solver_output), std::runtime_error);
    BOOST_CHECK(solver.getExitFlag() == DAQP_EXIT_INFEASIBLE);
}

BOOST_AUTO_TEST_CASE(solver_daqp_factory)
{
    std::shared_ptr<QPSolver> solver(QPSolverFactory::createInstance("daqp"));
    BOOST_CHECK(solver != nullptr);
    BOOST_CHECK(std::dynamic_pointer_cast<DaqpSolver>(solver) != nullptr);
}

BOOST_AUTO_TEST_CASE(solver_daqp_multiple_priorities_rejected)
{
    wbc::QuadraticProgram qp;
    qp.resize(2, 0, 0, false);
    qp.H.setIdentity();
    qp.g.setZero();
    wbc::HierarchicalQP hqp;
    hqp << qp;
    hqp << qp;

    DaqpSolver solver;
    Eigen::VectorXd solver_output;
    BOOST_CHECK_THROW(solver.solve(hqp, solver_output), std::invalid_argument);
}
