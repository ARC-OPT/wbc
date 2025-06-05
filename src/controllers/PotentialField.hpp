#pragma once

#include <Eigen/Core>
#include <stdexcept>
#include <memory>

namespace wbc{

/**
 *  @brief Base class for potential fields
 */
class PotentialField{
public:
    /** Init all members. Dimension of the field. Has to be > 0. e.g. a field in Cartesian space would have dimension 3. */
    PotentialField(const uint _dimension, const std::string& _name = "unset")
        : dimension(_dimension),
          name(_name){
        assert(dimension > 0);
        influence_distance = std::numeric_limits<double>::infinity(); //Initialize with inifinite maximum influence distance
        distance.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
        gradient.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
        pot_field_center.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
    }

    virtual ~PotentialField(){}

    /**
     * @brief Implement in derived class. Compute control update according to potential field equation.
     * @return Computed gradient. Size will be same as dimension
     */
    virtual const Eigen::VectorXd& update(const Eigen::VectorXd &position) = 0;

    /** Dimension of the potential field, e.g. a potential field in 3d space would have size 3.*/
    uint dimension;

    /** Maximum influence distance of the field. Default will be inf*/
    double influence_distance;

    /** Distance vector to the potential field. */
    Eigen::VectorXd distance;

    /** Potential field center position*/
    Eigen::VectorXd pot_field_center;

    /** Gradient for this field*/
    Eigen::VectorXd gradient;

    /** ID of the potential field*/
    const std::string name;
};

typedef std::shared_ptr<PotentialField> PotentialFieldPtr;

}

