#ifndef POTENTIAL_FIELD_INFO_HPP
#define POTENTIAL_FIELD_INFO_HPP

#include "PotentialField.hpp"

namespace ctrl_lib {

struct PotentialFieldInfo{

    void fromPotentialField(PotentialFieldPtr field){
        dimension = field->dimension;
        influence_distance = field->influence_distance;
        distance = field->distance;
        gradient = field->gradient;
        pot_field_center = field->pot_field_center;
        euclidean_distance = distance.norm();
        name = field->name;
    }

    /** Dimension of the potential field, e.g. a potential field in 3d space would have size 3.*/
    uint dimension;

    /** Maximum influence distance of the field. Default will be inf*/
    double influence_distance;

    /** Distance vector to the potential field. */
    base::VectorXd distance;

    /** Euclidean distance of the distance vector*/
    double euclidean_distance;

    /** Gradient vector for this field*/
    base::VectorXd gradient;

    /** Potential field center position*/
    base::VectorXd pot_field_center;

    /** ID of the potential field*/
    std::string name;
};


}

#endif
