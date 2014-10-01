#ifndef EXTENDED_CONSTRAINT_HPP
#define EXTENDED_CONSTRAINT_HPP

#include "Constraint.hpp"

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace wbc{

class ExtendedConstraint : public Constraint{
public:

    ExtendedConstraint(const ConstraintConfig& _config,
                       const std::vector<std::string> &_robot_joint_names) :
        Constraint(_config, _robot_joint_names)
    {
        uint no_vars;
        if(_config.type == jnt)
            no_vars = _config.joint_names.size();
        else
            no_vars = 6;
        init(no_vars);
    }

    ~ExtendedConstraint(){
    }

    void init(uint no_vars){
        full_jac = KDL::Jacobian(no_vars);
        full_jac.data.setZero();

        H.resize(no_vars,6);
        H.setZero();

        Uf.resize(6, no_vars);
        Uf.setIdentity();

        Vf.resize(no_vars, no_vars);
        Vf.setIdentity();

        Sf.resize(no_vars);
        Sf.setZero();
    }

    KDL::Jacobian full_jac;
    KDL::Frame pose;

    //Helpers for inversion of the Jacobian
    Eigen::MatrixXd Uf, Vf;
    Eigen::VectorXd Sf;
    Eigen::MatrixXd H;
};
}

#endif
