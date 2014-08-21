#ifndef SOLVERTYPES_HPP
#define SOLVERTYPES_HPP

#include <base/Eigen.hpp>
#include <vector>

namespace wbc{

/**
 * @brief Solver input per priority
 */
struct SolverInputPrio{
    SolverInputPrio(){}
    SolverInputPrio(uint ny, uint nx){
        A.resize(ny, nx);
        y_ref.resize(ny);
        Wy.resize(ny);

        A.setZero();
        y_ref.setZero();
        Wy.setIdentity(); //Set all task weights to 1 in the beginning
    }

    base::MatrixXd A;      /** Task Jacobians for one priority level */
    base::VectorXd Wy;     /** Task Weight Vector for one priority level */
    base::VectorXd y_ref;  /** Constraint variables */
};


/**
 * @brief Solver input for all priorities
 */
struct SolverInput{
    std::vector<std::string> joint_names;
    std::vector<SolverInputPrio> priorities;
};

}

#endif // SOLVERTYPES_HPP
