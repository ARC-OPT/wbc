#ifndef PRIORITY_DATA_HPP
#define PRIORITY_DATA_HPP

#include <base/Eigen.hpp>
#include <base/float.h>
#include <base/time.h>

namespace wbc{
/**
 * @brief The PriorityData class contains debug information for a priority level
 */
class PriorityData{
public:
    PriorityData(){}

    PriorityData(const unsigned int ny, const unsigned int nx, const unsigned int prio){
        priority = prio;
        no_constraints = ny;
        no_joints = nx;

        y_des.resize(ny);
        y_des.setConstant(base::NaN<double>());
        y_solution.resize(ny);
        y_solution.setConstant(base::NaN<double>());
        singular_vals.resize(nx);
        singular_vals.setConstant(base::NaN<double>());

        max_singular_val = base::NaN<double>();
        min_singular_val = base::NaN<double>();
        manipulability = base::NaN<double>();
        sqrt_wbc_err = base::NaN<double>();
        proj_time = base::NaN<double>();
        weighting_time = base::NaN<double>();
        svd_time = base::NaN<double>();
        compute_inverse_time = base::NaN<double>();
        total_time = base::NaN<double>();
        damping = base::NaN<double>();
    }
    base::Time time;
    uint priority;
    uint no_constraints;
    uint no_joints;
    base::VectorXd y_des;         /** Reference vector */
    base::VectorXd y_solution;    /** Solution per priority computed by the solver */
    base::VectorXd singular_vals; /** Singular values of A_proj_w_ */

    double max_singular_val; /** Biggest singular value*/
    double min_singular_val; /** Smallest singular value*/
    double manipulability; /** Manipulability index of the nullspace projected constraint mat of this priority, computed as M = det( J * J^T ) */
    double sqrt_wbc_err; /** Square root error between desired reference and computed solution*/
    double damping; /** Damping used on that priority for inverse computation */

    double proj_time; /** Computation time for nullspace projection */
    double weighting_time; /** Computation time for weighting */
    double svd_time; /** Computation time for singular value decomposition */
    double compute_inverse_time; /** Time for inverse computation */
    double total_time; /** Total Time for priority */
};
}

#endif
