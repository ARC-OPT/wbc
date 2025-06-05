#pragma once

#include "PotentialField.hpp"

namespace wbc{

/**
 * @brief Radial Potential field. The computed gradient will be constant on volumnes with
 *        constant radius around the center of the potential field:
 *
 *  grad = (x - x0) / ||d||^{2},  ||d|| <= dMax
 *           = 0                 ,  else
 *
 *  with: d     = x - x0 = Distance to field
 *        dMax  = Maximum Influence distance
 *        x     = current position
 *        x0    = Potential field center
 */
class RadialPotentialField : public PotentialField{
public:
    /** Init all members. Dimension of the field. Has to be > 0. e.g. a field in Cartesian space would have dimension 3. */
    RadialPotentialField(const uint _dimension, const std::string& _name = "");
    virtual ~RadialPotentialField(){}
    /**
     * @brief Compute control update according to potential field equation
     * @return Computed gradient. Size will be same as dimension.
     */
    virtual const Eigen::VectorXd& update(const Eigen::VectorXd& position);
};
}
