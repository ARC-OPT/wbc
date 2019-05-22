#include "QuadraticProgram.hpp"

namespace wbc {

void QuadraticProgram::resize(const uint nc, const uint nq){
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

}
