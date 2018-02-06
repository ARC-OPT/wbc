#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>

namespace wbc{

/**
 * @brief Compute the twist that is required to transform the given frame a onto the given frame b
 * @param a Actual pose
 * @param b Target pose
 * @param dt Time diff
 * @param twist Twist vector
 */
void pose_diff(const base::samples::RigidBodyState& a, const base::samples::RigidBodyState& b, const double dt, base::Vector6d& twist);

/**
 * @brief Compute the twist that is required to transform the given frame a onto the given frame b
 * @param a Actual pose
 * @param b Target pose
 * @param dt Time diff
 * @param twist Twist vector
 */
void pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b, const double dt, base::Vector6d& twist);

} // namespace wbc

#endif // TOOLS_HPP
