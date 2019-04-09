#include "Acceleration.hpp"

namespace wbc {

Acceleration::Acceleration(){
    setNaN();
}

void Acceleration::setNaN(){
    linear.setConstant(std::numeric_limits<double>::quiet_NaN());
    angular.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Acceleration::setZero(){
    linear.setZero();
    angular.setZero();
}

bool Acceleration::isValid() const{
    return base::isnotnan(linear) && base::isnotnan(angular);
}

Acceleration operator+(const Acceleration& a, const Acceleration& b){
    Acceleration c;
    c.linear  = a.linear  + b.linear;
    c.angular = a.angular + b.angular;
    return c;
}

Acceleration operator*(const base::Vector6d& a, const Acceleration& b){
    Acceleration c;
    c.linear  = a.segment(0,3).cwiseProduct(b.linear);
    c.angular = a.segment(3,3).cwiseProduct(b.angular);
    return c;
}

}
