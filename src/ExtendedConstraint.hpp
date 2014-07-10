#ifndef EXTENDED_CONSTRAINT_HPP
#define EXTENDED_CONSTRAINT_HPP

#include "Constraint.hpp"

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace wbc{

class ExtendedConstraint : public Constraint{
public:

    ExtendedConstraint(const ConstraintConfig& _config,
                    const uint _no_robot_joints,
                    bool _constraints_active  = true) :
        Constraint(_config, _no_robot_joints, _constraints_active)
    {
        if(_config.type == jnt)
            no_variables = _config.joint_names.size();
        else
            no_variables = 6;
       init(no_variables);
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
