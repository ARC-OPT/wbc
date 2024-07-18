#pragma once

#include "PotentialField.hpp"

namespace wbc{

class PotentialField;

/**
 * @brief Planar Potential field. The gradient will be constant on planes parallel to the
 *        plane defined by x0 (origin) and n (surface normal)
 *
 *  grad = (||d||*n) / ||d||^{2},  ||d|| <= d_0
 *       = 0                      ,  else
 *
 *  with: d     = |n*(x - x0)| / ||n|| = Distance to the plane defined by x0 and n
 *        n     = Plain normal
 *        dMax  = Maximum Influence distance
 *        x     = current position
 *        x0    = Plain support vector field center
 */
class PlanarPotentialField : public PotentialField{
public:

    /** Init all members. Dimension is 3!*/
    PlanarPotentialField(const std::string &_name = "");
    virtual ~PlanarPotentialField(){}

    /**
     * @brief Compute control update according to potential field equation
     * @param gradient Computed gradient. Will be resized if gradient.size() != dimension.
     */
    virtual const Eigen::VectorXd& update(const Eigen::VectorXd& position);

    /** Normal that defines the orientation of the plane*/
    Eigen::VectorXd n;
};

typedef std::shared_ptr<PlanarPotentialField> PlanarPotentialFieldPtr;

}

