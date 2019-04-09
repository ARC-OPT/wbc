#include "Twist.hpp"

namespace wbc {

Twist::Twist(){
    setNaN();
}

void Twist::setNaN(){
    linear.setConstant(std::numeric_limits<double>::quiet_NaN());
    angular.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Twist::setZero(){
    linear.setZero();
    angular.setZero();
}

bool Twist::isValid() const{
    return base::isnotnan(linear) && base::isnotnan(angular);
}

Twist operator+(const Twist& a, const Twist& b){
    Twist c;
    c.linear  = a.linear  + b.linear;
    c.angular = a.angular + b.angular;
    return c;
}

Twist operator*(const base::Vector6d& a, const Twist& b){
    Twist c;
    c.linear  = a.segment(0,3).cwiseProduct(b.linear);
    c.angular = a.segment(3,3).cwiseProduct(b.angular);
    return c;
}

}
