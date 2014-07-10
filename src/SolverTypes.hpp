#ifndef SOLVERTYPES_HPP
#define SOLVERTYPES_HPP

#include <base/Eigen.hpp>

namespace wbc{
enum svd_method{svd_eigen, svd_kdl};

struct SolverInput{
    SolverInput(){}
    SolverInput(uint ny, uint nx){
        A.resize(ny, nx);
        y_ref.resize(ny);
        Wy.resize(ny);

        A.setZero();
        y_ref.setZero();
        Wy.setIdentity(); //Set all task weights to 1 in the beginning
    }

    base::MatrixXd A;  /** Prioritized equation system constructed from Task Jacobians */
    base::VectorXd Wy; /** Task Weight Vectors per priority */
    base::VectorXd y_ref;  /** Constraint variables */
};
}
#endif // SOLVERTYPES_HPP
