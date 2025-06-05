#include "PlanarPotentialField.hpp"

using namespace wbc;

PlanarPotentialField::PlanarPotentialField(const std::string &_name)
    : PotentialField(3, _name){
}

const Eigen::VectorXd& PlanarPotentialField::update(const Eigen::VectorXd& position){
    assert(position.size() == dimension);
    assert(pot_field_center.size() == dimension);
    assert(n.size() == dimension);

    gradient.resize(dimension);

    //Distance to the plane is:
    double dist_sqrt = fabs( n.dot(position - pot_field_center)) / n.norm();

    //Vector, which is normal to the plane and has length d == distance vector to the plane
    distance = dist_sqrt*n;

    assert(influence_distance > 0);

    if(dist_sqrt > influence_distance)
        gradient.setZero();
    else
    {
        if(dist_sqrt == 0)
            throw std::runtime_error("Distance to Potential Field Center is 0! This is an invalid state of the potential field!");
        else // Compute gradient ~ 1/dist_sqrt
            gradient = distance/(dist_sqrt*dist_sqrt);
    }

    return gradient;
}

