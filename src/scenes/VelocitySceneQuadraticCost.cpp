#include "VelocitySceneQuadraticCost.hpp"

#include <base/JointLimits.hpp>
#include <base-logging/Logging.hpp>

#include "../tasks/CartesianVelocityConstraint.hpp"
#include "../tasks/JointVelocityConstraint.hpp"

#include "../constraints/ContactsVelocityHardConstraint.hpp"
#include "../constraints/JointLimitsVelocityHardConstraint.hpp"

namespace wbc{

VelocitySceneQuadraticCost::VelocitySceneQuadraticCost(RobotModelPtr robot_model, QPSolverPtr solver, double dt) :
    VelocityScene(robot_model, solver),
    hessian_regularizer(1e-8){

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    hard_constraints.resize(1);
    hard_constraints[0].push_back(std::make_shared<ContactsVelocityHardConstraint>());
    hard_constraints[0].push_back(std::make_shared<JointLimitsVelocityHardConstraint>(dt));

}

VelocitySceneQuadraticCost::~VelocitySceneQuadraticCost(){
}

const HierarchicalQP& VelocitySceneQuadraticCost::update(){

    if(!configured)
        throw std::runtime_error("VelocitySceneQuadraticCost has not been configured!. Please call configure() before calling update() for the first time!");

    if(constraints.size() != 1){
        LOG_ERROR("Number of priorities in VelocitySceneQuadraticCost should be 1, but is %i", constraints.size());
        throw std::runtime_error("Invalid constraint configuration");
    }

    int nj = robot_model->noOfJoints();
    const ActiveContacts& contact_points = robot_model->getActiveContacts();
    uint ncp = contact_points.size();
    uint prio = 0;

    // QP Size: (NContacts*6 X NJoints)
    constraints_prio[prio].resize(ncp*6,nj);
    constraints_prio[prio].H.setZero();
    constraints_prio[prio].g.setZero();

    ///////// Tasks

    for(uint i = 0; i < constraints[prio].size(); i++){
        
        ConstraintPtr constraint = constraints[prio][i];
        
        constraint->checkTimeout();
        constraint->update(robot_model);

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
        if(constraint->activation == 0){
           constraint->y_ref.setZero();
           constraint->y_ref_root.setZero();
        }

        for(int i = 0; i < constraint->A.rows(); i++)
            constraint->Aw.row(i) = constraint->weights_root(i) * constraint->A.row(i) * constraint->activation * (!constraint->timeout);
        for(int i = 0; i < constraint->A.cols(); i++)
            constraint->Aw.col(i) = joint_weights[i] * constraint->Aw.col(i);

        constraints_prio[prio].H.block(0,0,nj,nj) += constraint->Aw.transpose()*constraint->Aw;
        constraints_prio[prio].g.segment(0,nj) -= constraint->Aw.transpose()*constraint->y_ref_root;

    } // constraints on prio

    // Add regularization term
    constraints_prio[prio].H.block(0,0,nj,nj).diagonal().array() += hessian_regularizer;


    ///////// Constraints

    size_t total_eqs = 0;
    for(auto hard_contraint : hard_constraints[prio]) {
        hard_contraint->update(robot_model);
        if(hard_contraint->type() != HardConstraint::bounds)
            total_eqs += hard_contraint->size();
    }

    // Note already performed at the beginning of the update (but does not consider additional constriants)
    constraints_prio[prio].A.resize(total_eqs, nj);
    constraints_prio[prio].lower_x.resize(nj);
    constraints_prio[prio].upper_y.resize(nj);
    constraints_prio[prio].lower_y.resize(total_eqs);
    constraints_prio[prio].upper_y.resize(total_eqs);

    constraints_prio[prio].A.setZero();
    constraints_prio[prio].lower_y.setConstant(-99999);
    constraints_prio[prio].upper_y.setConstant(+99999);
    constraints_prio[prio].lower_x.setConstant(-99999);
    constraints_prio[prio].upper_x.setConstant(+99999);

    total_eqs = 0;
    for(uint i = 0; i < hard_constraints[prio].size(); i++) {
        HardConstraint::Type type = hard_constraints[prio][i]->type();
        size_t c_size = hard_constraints[prio][i]->size();

        if(type == HardConstraint::bounds) {
            constraints_prio[prio].lower_x = hard_constraints[prio][i]->lb();
            constraints_prio[prio].upper_x = hard_constraints[prio][i]->ub();
        }
        else if (type == HardConstraint::equality) {
            constraints_prio[prio].A.middleRows(total_eqs, c_size) = hard_constraints[prio][i]->A();
            constraints_prio[prio].lower_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->b();
            constraints_prio[prio].upper_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->b();
        }
        else if (type == HardConstraint::inequality) {
            constraints_prio[prio].A.middleRows(total_eqs, c_size) = hard_constraints[prio][i]->A();
            constraints_prio[prio].lower_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->lb();
            constraints_prio[prio].upper_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->ub();
        }

        total_eqs += c_size;
    }

    // // For all contacts: Js*qd = 0 (Rigid Contacts, contact points do not move!)
    // constraints_prio[prio].A.setZero();
    // for(int i = 0; i < contact_points.size(); i++)
    //     constraints_prio[prio].A.block(i*6, 0, 6, nj) = contact_points[i]*robot_model->bodyJacobian(robot_model->baseFrame(), contact_points.names[i]);
    // constraints_prio[prio].lower_y.setZero();
    // constraints_prio[prio].upper_y.setZero();
    // // TODO: Using actual limits does not work well (QP Solver sometimes fails due to infeasible QP)
    // constraints_prio[prio].lower_x.setConstant(-1000);
    // constraints_prio[prio].upper_x.setConstant(1000);
    // for(auto n : robot_model->actuatedJointNames()){
    //     size_t idx = robot_model->jointIndex(n);
    //     const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);
    //     constraints_prio[prio].lower_x(idx) = range.min.speed;
    //     constraints_prio[prio].upper_x(idx) = range.max.speed;
    // }

    constraints_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all constraints!?

    return constraints_prio;
}

} // namespace wbc
