#include "RadialPotentialField.hpp"

using namespace ctrl_lib;

RadialPotentialField::RadialPotentialField(const uint _dimension, const std::string& _name)
    : PotentialField(_dimension, _name){
}

 const base::VectorXd& RadialPotentialField::update(const base::VectorXd& position){
    if(position.size() != dimension)
        throw std::invalid_argument("RadialPotentialField::update: Size of actual position vector must be equal to controller dimension");
    if(pot_field_center.size() != dimension)
        throw std::invalid_argument("RadialPotentialField::update: Size of potential field center vector must be equal to controller dimension");

    gradient.resize(dimension);

    //Compute euclidean distance to field center
    distance = position - pot_field_center;
    double dist_sqrt = distance.norm();

    if(influence_distance <= 0)
        throw std::invalid_argument("RadialPotentialField::update: influence_distance has to be > 0!");


    if(dist_sqrt > influence_distance)
        gradient.setZero();
    else
    {
        if(dist_sqrt != 0){ // Sigmoid function

            // Normalize to 0..1
            //   If distance -> 0,                   normalized distance -> 1
            //   If distance -> influence_distance , normalized distance -> 0
            double normalized_dist = (influence_distance - dist_sqrt) / influence_distance;

            // if normalized_dist --> 0, factor --> 1
            // if normalized_dist == influence_distance/2, factor = 0.5
            // if dist --> influence_distance, factor --> 0
            double factor = 1 / (1 + exp( (1 - 2 * normalized_dist ) * 6));

            // multiple with normalize distance vector to get correct gradient direction
            gradient = factor * (distance / dist_sqrt);
        }
    }

    return gradient;
}
