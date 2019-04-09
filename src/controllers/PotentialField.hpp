#pragma once

#include <base/Eigen.hpp>
#include <stdexcept>
#include <memory>

namespace ctrl_lib{

/**
 *  @brief Base class for potential fields
 */
class PotentialField{
public:
    /** Init all members. Dimension of the field. Has to be > 0. e.g. a field in Cartesian space would have dimension 3. */
    PotentialField(const uint _dimension, const std::string& _name = "unset")
        : dimension(_dimension),
          name(_name){
        if(dimension == 0)
            throw std::invalid_argument("PotentialField:PotentialField: Dimension of Potential Field has to be > 0");

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
    virtual const base::VectorXd& update(const base::VectorXd &position) = 0;

    /** Dimension of the potential field, e.g. a potential field in 3d space would have size 3.*/
    uint dimension;

    /** Maximum influence distance of the field. Default will be inf*/
    double influence_distance;

    /** Distance vector to the potential field. */
    base::VectorXd distance;

    /** Potential field center position*/
    base::VectorXd pot_field_center;

    /** Gradient for this field*/
    base::VectorXd gradient;

    /** ID of the potential field*/
    const std::string name;
};

typedef std::shared_ptr<PotentialField> PotentialFieldPtr;

}

