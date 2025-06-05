#include "RadialPotentialField.hpp"

using namespace wbc;

RadialPotentialField::RadialPotentialField(const uint _dimension, const std::string& _name)
    : PotentialField(_dimension, _name){
}

 const Eigen::VectorXd& RadialPotentialField::update(const Eigen::VectorXd& position){
    assert(position.size() == dimension);
    assert(pot_field_center.size() == dimension);
    assert(influence_distance > 0);

    gradient.resize(dimension);

    //Compute euclidean distance to field center
    distance = position - pot_field_center;
    double dist_sqrt = distance.norm();

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
