#include "VelocitySceneQuadraticCost.hpp"
#include <base/JointLimits.hpp>
#include <base-logging/Logging.hpp>
#include "../core/CartesianVelocityConstraint.hpp"
#include "../core/JointVelocityConstraint.hpp"

namespace wbc{

VelocitySceneQuadraticCost::VelocitySceneQuadraticCost(RobotModelPtr robot_model, QPSolverPtr solver) :
    VelocityScene(robot_model, solver){

}

VelocitySceneQuadraticCost::~VelocitySceneQuadraticCost(){
}

const HierarchicalQP& VelocitySceneQuadraticCost::update(){

    if(!configured)
        throw std::runtime_error("VelocitySceneQuadraticCost has not been configured!. PLease call configure() before calling update() for the first time!");

    if(constraints.size() != 1){
        LOG_ERROR("Number of priorities in VelocitySceneQuadraticCost should be 1, but is %i", constraints.size());
        throw std::runtime_error("Invalid constraint configuration");
    }

    int nj = robot_model->noOfJoints();
    std::vector<std::string> contact_points = robot_model->getContactPoints();
    uint ncp = contact_points.size();
    uint prio = 0;

    constraints_prio[prio].resize(ncp*6,nj);

    A.resize(n_constraint_variables_per_prio[prio],nj);
    A.setZero();
    A_weighted.resize(n_constraint_variables_per_prio[prio],nj);
    y.resize(n_constraint_variables_per_prio[prio]);
    Wy.resize(n_constraint_variables_per_prio[prio]);

    // Walk through all tasks of current priority
    uint row_index = 0;
    for(uint i = 0; i < constraints[prio].size(); i++){

        constraints[prio][i]->checkTimeout();
        int type = constraints[prio][i]->config.type;
        uint n_vars = constraints[prio][i]->config.nVariables();

        if(type == cart){

            CartesianVelocityConstraintPtr constraint = std::static_pointer_cast<CartesianVelocityConstraint>(constraints[prio][i]);

            // Constraint Jacobian
            constraint->A = robot_model->spaceJacobian(constraint->config.root, constraint->config.tip);

            // Convert constraint twist to robot root
            base::MatrixXd rot_mat = robot_model->rigidBodyState(constraint->config.root, constraint->config.ref_frame).pose.orientation.toRotationMatrix();
            constraint->y_ref_root.segment(0,3) = rot_mat * constraint->y_ref.segment(0,3);
            constraint->y_ref_root.segment(3,3) = rot_mat * constraint->y_ref.segment(3,3);

            // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
            // assume positive values
            constraint->weights_root.segment(0,3) = rot_mat * constraint->weights.segment(0,3);
            constraint->weights_root.segment(3,3) = rot_mat * constraint->weights.segment(3,3);
            constraint->weights_root = constraint->weights_root.cwiseAbs();
        }
        else if(type == jnt){

            JointVelocityConstraintPtr constraint = std::static_pointer_cast<JointVelocityConstraint>(constraints[prio][i]);

            // Joint space constraints: constraint matrix has only ones and Zeros. The joint order in the constraints might be different than in the robot model.
            // Thus, for joint space constraints, the joint indices have to be mapped correctly.
            for(uint k = 0; k < constraint->config.joint_names.size(); k++){

                int idx = robot_model->jointIndex(constraint->config.joint_names[k]);
                constraint->A(k,idx) = 1.0;
                constraint->y_ref_root = constraint->y_ref;     // In joint space y_ref is equal to y_ref_root
                constraint->weights_root = constraint->weights; // Same of the weights
            }
        }
        else{
            LOG_ERROR("Constraint %s: Invalid type: %i", constraints[prio][i]->config.name.c_str(), type);
            throw std::invalid_argument("Invalid constraint configuration");
        }

        ConstraintPtr constraint = constraints[prio][i];

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
        if(constraint->activation == 0){
           constraint->y_ref.setZero();
           constraint->y_ref_root.setZero();
        }

        // Insert constraints into equation system of current priority at the correct position. Note: Weights will be zero if activations
        // for this constraint is zero or if the constraint is in timeout
        Wy.segment(row_index, n_vars) = constraint->weights_root * constraint->activation;// * (!constraint->timeout);
        A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = constraint->A;
        y.segment(row_index, n_vars) = constraint->y_ref_root;

        row_index += n_vars;

    } // constraints on prio


    ///////// Tasks

    // Mutiply Task weights
    for(int i = 0; i < A.rows(); i++)
        A_weighted.row(i) = Wy(i) * A.row(i);
    // Mutiply joint weights
    for(int i = 0; i < A.cols(); i++)
        A_weighted.col(i) = joint_weights[i] * A_weighted.col(i);

    // Compute Hessian: H = A^T*A
    constraints_prio[prio].H = A_weighted.transpose()*A_weighted;
    // Add regularization term
    constraints_prio[prio].H = constraints_prio[prio].H + 1e-9*base::MatrixXd::Identity(nj,nj);
    // gradient vector: -(A^T*y)^T
    constraints_prio[prio].g = -(A_weighted.transpose()*y);


    ///////// Constraints

    // For all contacts: Js*qd = 0 (Rigid Contacts, contact points do not move!)
    constraints_prio[prio].A.setZero();
    for(int i = 0; i < contact_points.size(); i++)
        constraints_prio[prio].A.block(i*6, 0, 6, nj) = robot_model->bodyJacobian(robot_model->baseFrame(), contact_points[i]);
    constraints_prio[prio].lower_y.setZero();
    constraints_prio[prio].upper_y.setZero();
    // No velocity limits. Using actual limits does not work well here. QP Solver sometimes fails due to infeasible QP
    constraints_prio[prio].lower_x.resize(0);
    constraints_prio[prio].upper_x.resize(0);
    /*for(auto n : robot_model->actuatedJointNames()){
        size_t idx = robot_model->jointIndex(n);
        const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);
        constraints_prio[prio].lower_x(idx) = -range.min.speed;
        constraints_prio[prio].upper_x(idx) = range.max.speed;
    }*/

    return constraints_prio;
}

} // namespace wbc
