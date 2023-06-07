#include "QPSolver.hpp"

namespace wbc{

QPSolver::QPSolver() : configured(false){
}

QPSolver::~QPSolver(){
}

QPSolverFactory::QPSolverMap* QPSolverFactory::qp_solver_map = 0;
}
