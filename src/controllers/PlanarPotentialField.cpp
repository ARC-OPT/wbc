#include "PlanarPotentialField.hpp"

using namespace ctrl_lib;

PlanarPotentialField::PlanarPotentialField(const std::string &_name)
    : PotentialField(3, _name){
}

const base::VectorXd& PlanarPotentialField::update(const base::VectorXd& position){
    if(position.size() != dimension)
        throw std::invalid_argument("PlanarPotentialField::update: Size of actual position vector must be equal to controller dimension");
    if(pot_field_center.size() != dimension)
        throw std::invalid_argument("PlanarPotentialField::update: Size of potential field center vector must be equal to controller dimension");
    if(n.size() != dimension)
        throw std::invalid_argument("PlanarPotentialField::update: Size of plane normal vector must be equal to controller dimension");

    gradient.resize(dimension);

    //Distance to the plane is:
    double dist_sqrt = fabs( n.dot(position - pot_field_center)) / n.norm();

    //Vector, which is normal to the plane and has length d == distance vector to the plane
    distance = dist_sqrt*n;

    if(influence_distance <= 0)
        throw std::invalid_argument("PlanarPotentialField::update: influence_distance has to be > 0!");

    if(dist_sqrt > influence_distance)
        gradient.setZero();
    else
    {
        if(dist_sqrt == 0)
            throw std::runtime_error("Distance to Potential Field Center is 0! This is an invalid state of your system!");
        else // Compute gradient ~ 1/dist_sqrt
            gradient = distance/(dist_sqrt*dist_sqrt);
    }

    return gradient;
}

