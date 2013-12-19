#include <boost/test/unit_test.hpp> 
#include "../src/HierarchicalWDLSSolver.hpp"
#include <stdlib.h>

using namespace std;


BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
   srand (time(NULL));

   const int NO_JOINTS = 2;
   const int NO_CONSTRAINTS = 3;

   HierarchicalWDLSSolver solver;
   std::vector<unsigned int> ny_per_prio(1,NO_CONSTRAINTS);
   solver.configure(ny_per_prio, NO_JOINTS);

   Eigen::MatrixXd A(NO_CONSTRAINTS,NO_JOINTS);
   A.setIdentity();
   for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ ) A.data()[i] = (rand()%1000)/1000.0;
   Eigen::VectorXd y(NO_CONSTRAINTS);
   y.setConstant(1);

   std::vector<Eigen::MatrixXd> A_prio;
   A_prio.push_back(A);

   std::vector<Eigen::VectorXd> y_prio;
   y_prio.push_back(y);

   cout<<"............Testing Hierarchical Solver "<<endl<<endl;
   cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
   cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
   cout<<"No of joints: "<<NO_JOINTS<<endl;
   cout<<"Solver Input: "<<endl;
   for(uint i = 0; i < ny_per_prio.size(); i++){
       cout<<"Priority: "<<i<<endl;
       cout<<"A: "<<endl; cout<<A_prio[i]<<endl;
       cout<<"y: "<<endl; cout<<y_prio[i]<<endl;
       cout<<endl;
   }

   Eigen::VectorXd solver_output;
   solver.solve(A_prio, y_prio, solver_output);
   cout<<"Output: "<<solver_output<<endl;

   cout<<"Test: "<<endl;
   for(uint i = 0; i < ny_per_prio.size(); i++){
       cout<<"Priority: "<<i<<endl;
       cout<<"A*q: "<<endl; cout<<A_prio[i]*solver_output<<endl;
       cout<<endl;
   }
}
