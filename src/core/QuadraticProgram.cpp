#include "QuadraticProgram.hpp"
#include <iostream>

namespace wbc {

void QuadraticProgram::resize(const uint _nq, const uint _neq, const uint _nin, bool _bounds){
    neq = _neq;
    nin = _nin;
    nq = _nq;

    bounds = _bounds;

    // cost function
    H.resize(nq, nq);
    H.setConstant(std::numeric_limits<double>::quiet_NaN());
    g.resize(nq);
    g.setConstant(std::numeric_limits<double>::quiet_NaN());

    // equalities
    A.resize(neq, nq);
    A.setConstant(std::numeric_limits<double>::quiet_NaN());
    b.resize(neq);
    b.setConstant(std::numeric_limits<double>::quiet_NaN());

    // inequalities
    C.resize(nin, nq);
    C.setConstant(std::numeric_limits<double>::quiet_NaN());
    lower_y.resize(nin);
    lower_y.setConstant(std::numeric_limits<double>::quiet_NaN());
    upper_y.resize(nin);
    upper_y.setConstant(std::numeric_limits<double>::quiet_NaN());

    // bounds
    lower_x.resize(bounds ? nq : 0);
    lower_x.setConstant(std::numeric_limits<double>::quiet_NaN());
    upper_x.resize(bounds ? nq : 0);
    upper_x.setConstant(std::numeric_limits<double>::quiet_NaN());

    Wy.setOnes(neq+nin);
}

void QuadraticProgram::check() const {
    if(bounds) {
        if(lower_x.size() != nq)
            throw std::runtime_error("Quadratic program with " + std::to_string(nq) + " variables has bounds "
                + "but lower bound has size " + std::to_string(lower_x.size()));
        if(upper_x.size() != nq)
            throw std::runtime_error("Quadratic program with " + std::to_string(nq) + " variables has bounds "
                + "but upper bound has size " + std::to_string(upper_x.size()));
    }
    else {
        if(lower_x.size() != 0)
            throw std::runtime_error("Quadratic program has not bounds "
                "but lower bound has size " + std::to_string(lower_x.size()) + " (nq:" + std::to_string(nq) + ")");
        if(upper_x.size() != 0)
            throw std::runtime_error("Quadratic program has not bounds "
                "but upper bound has size " + std::to_string(lower_x.size()) + " (nq:" + std::to_string(nq) + ")");
    }
    if(C.rows() != nin || C.cols() != nq)
        throw std::runtime_error("Inequality constraint matrix C should have size " + std::to_string(nin) + "x" + std::to_string(nq) +
            "but has size " +  std::to_string(C.rows()) + "x" + std::to_string(C.cols()));
    if(lower_y.size() != nin)
        throw std::runtime_error("Number of inequality constraints in quadratic program is " + std::to_string(nin)
            + ", but lower bound has size " + std::to_string(lower_y.size()));
    if(upper_y.size() != nin)
        throw std::runtime_error("Number of inequality constraints in quadratic program is " + std::to_string(nin)
            + ", but lower bound has size " + std::to_string(lower_y.size()));
    if(A.rows() != neq || A.cols() != nq)
        throw std::runtime_error("Equality constraint matrix A should have size " + std::to_string(neq) + "x" + std::to_string(nq) +
            "but has size " +  std::to_string(A.rows()) + "x" + std::to_string(A.cols()));
    if(b.size() != neq)
            throw std::runtime_error("Equality constraint vector b should have size " + std::to_string(neq) + "but has size " + std::to_string(b.size()));
    if(H.rows() != nq || H.cols() != nq)
        throw std::runtime_error("Hessian matrix H should have size " + std::to_string(nq) + "x" + std::to_string(nq) +
            "but has size " +  std::to_string(H.rows()) + "x" + std::to_string(H.cols()));
    if(g.size() != nq)
        throw std::runtime_error("Gradient vector g should have size " + std::to_string(nq) + "but has size " + std::to_string(g.size()));
}

void QuadraticProgram::print() const {
    std::cout << "-- Quadratic Program --" << std::endl;
    std::cout << "Size nq: " << nq << "  neq: " << neq << "  nin:" << nin << std::endl;
    std::cout << "H" << std::endl;
    std::cout << H << std::endl;
    std::cout << "g" << std::endl;
    std::cout << g.transpose() << std::endl;
    std::cout << "A" << std::endl;
    std::cout << A << std::endl;
    std::cout << "b" << std::endl;
    std::cout << b.transpose() << std::endl;
    std::cout << "C" << std::endl;
    std::cout << C << std::endl;
    std::cout << "lower_y" << std::endl;
    std::cout << lower_y.transpose() << std::endl;
    std::cout << "upper_y" << std::endl;
    std::cout << upper_y.transpose() << std::endl;
    std::cout << "lower_x" << std::endl;
    std::cout << lower_x.transpose() << std::endl;
    std::cout << "upper_x" << std::endl;
    std::cout << upper_x.transpose() << std::endl;
    std::cout << "Wy" << std::endl;
    std::cout << Wy.transpose() << std::endl;
}

}
