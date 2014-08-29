#ifndef SOLVERTYPES_HPP
#define SOLVERTYPES_HPP

#include <base/Eigen.hpp>
#include <vector>
#include "Constraint.hpp"

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

    base::MatrixXd A;      /** m x n Task Jacobian matrix for one priority level */
    base::VectorXd Wy;     /** m x 1 Task Weight Vector for one priority level */
    base::VectorXd y_ref;  /** m x 1 Constraint variables for one priority level */

};


/**
 * @brief Solver input for all priorities
 */
struct SolverInput{
    std::vector<std::string> joint_names;
    std::vector<SolverInputPrio> priorities; /** Priority vector, ordered by priority, 0 = highest */
    base::VectorXd Wx;                       /** n x 1 Joint weight vector */
};

}

#endif // SOLVERTYPES_HPP
