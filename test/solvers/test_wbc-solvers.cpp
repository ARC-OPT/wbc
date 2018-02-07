#include <boost/test/unit_test.hpp>
#include "solvers/HierarchicalLSSolver.hpp"
#include "solvers/HierarchicalLSOptProblem.hpp"
#include <iostream>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(hierarchical_ls_solver)
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

    HierarchicalLSOptProblem input;
    LinearEqualityConstraints prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        prio_0.A.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.y_ref.data()[i] = (rand()%1000)/1000.0;
    prio_0.W.setConstant(1);

    input.push_back(prio_0);

    cout<<"\n----------------------- Testing Hierarchical Solver ----------------------"<<endl<<endl;
    cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
    cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;

    cout<<"\nSolver Input ..."<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"Priority: "<<i<<endl;
        cout<<"Constraint Matrix: "<<endl; cout<<prio_0.A<<endl;
        cout<<"Reference: "<<endl; cout<<prio_0.y_ref<<endl;
        cout<<endl;
    }

    base::VectorXd solver_output;
    solver.solve(input, solver_output);

    cout<<"\nSolver Output ..."<<endl;
    cout<<"q_dot = "<<endl;
    cout<<solver_output<<endl;

    cout<<"\nTEST..."<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<" ---- Priority: "<<i<<" ----"<<endl<<endl;
        Eigen::VectorXd test = prio_0.A*solver_output;
        cout<<"A * q_dot: "<<endl;
        cout<<test<<endl; cout<<endl;
        for(uint j = 0; j < NO_CONSTRAINTS; j++)
            BOOST_CHECK(fabs(test(j) - prio_0.y_ref(j)) < 1e-9);
    }

    cout<<"\n............................."<<endl;
}
