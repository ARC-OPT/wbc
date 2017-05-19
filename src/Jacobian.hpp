#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <base/Eigen.hpp>

namespace wbc{

class Jacobian : public base::MatrixXd{
public:
    Jacobian();
    Jacobian(uint n_robot_joints);
    void changeRefPoint(const base::Vector3d& v);
    void changeRefFrame(const base::Affine3d& a);

};

} // namespace wbc

#endif // JACOBIAN_HPP
