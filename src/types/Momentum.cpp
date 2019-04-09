#include "Momentum.hpp"

namespace wbc{

void Momentum::setNaN(){
    linear.setConstant(std::numeric_limits<double>::quiet_NaN());
    angular.setConstant(std::numeric_limits<double>::quiet_NaN());
}

bool Momentum::hasValidLinearMomentum() const{
    return base::isnotnan(linear);
}

bool Momentum::hasValidAngularMomentum() const{
    return base::isnotnan(linear);
}

bool Momentum::isValid() const{
    return hasValidLinearMomentum() && hasValidAngularMomentum();
}

}
