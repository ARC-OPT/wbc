#include "QuadraticProgram.hpp"
#include <iostream>

namespace wbc {

void QuadraticProgram::resize(const uint _nc, const uint _nq){
    nc = _nc;
    nq = _nq;
    A.resize(nc, nq);
    A.setConstant(std::numeric_limits<double>::quiet_NaN());

    g.resize(nq);
    g.setConstant(std::numeric_limits<double>::quiet_NaN());

    lower_x.resize(nq);
    lower_x.setConstant(std::numeric_limits<double>::quiet_NaN());
    upper_x.resize(nq);
    upper_x.setConstant(std::numeric_limits<double>::quiet_NaN());

    lower_y.resize(nc);
    lower_y.setConstant(std::numeric_limits<double>::quiet_NaN());
    upper_y.resize(nc);
    upper_y.setConstant(std::numeric_limits<double>::quiet_NaN());

    H.resize(nq, nq);
    H.setConstant(std::numeric_limits<double>::quiet_NaN());

    Wy.setOnes(nc);
}

void QuadraticProgram::print() const{
    std::cout<<"-- Quadratic Program --"<<std::endl;
    std::cout<<"Size "<<nc<<" X "<<nq<<std::endl;
    std::cout<<"A"<<std::endl;
    std::cout<<A<<std::endl;
    std::cout<<"H"<<std::endl;
    std::cout<<H<<std::endl;
    std::cout<<"g"<<std::endl;
    std::cout<<g.transpose()<<std::endl;
    std::cout<<"lower_x"<<std::endl;
    std::cout<<lower_x.transpose()<<std::endl;
    std::cout<<"upper_x"<<std::endl;
    std::cout<<upper_x.transpose()<<std::endl;
    std::cout<<"lower_y"<<std::endl;
    std::cout<<lower_y.transpose()<<std::endl;
    std::cout<<"upper_y"<<std::endl;
    std::cout<<upper_y.transpose()<<std::endl;
    std::cout<<"Wy"<<std::endl;
    std::cout<<Wy.transpose()<<std::endl;
}

}
