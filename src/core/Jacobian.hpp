#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <base/Eigen.hpp>

namespace wbc{

/**
 * @brief Helper class to represent a Jacobian.
 */
class Jacobian : public base::MatrixXd{
public:
    Jacobian();
    Jacobian(uint n_robot_joints);
    ~Jacobian();
    void changeRefPoint(const base::Vector3d& v);
    void changeRefFrame(const base::Affine3d& a);

};

} // namespace wbc

#endif // JACOBIAN_HPP
