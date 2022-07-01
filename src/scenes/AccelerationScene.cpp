#include "AccelerationScene.hpp"
#include "../core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

ConstraintPtr AccelerationScene::createConstraint(const ConstraintConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianAccelerationConstraint>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointAccelerationConstraint>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Constraint with name %s has an invalid constraint type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid constraint config");
    }
}

const HierarchicalQP& AccelerationScene::update(){

    if(!configured)
        throw std::runtime_error("AccelerationScene has not been configured!. PLease call configure() before calling update() for the first time!");

    if(constraints.size() != 1){
        LOG_ERROR("Number of priorities in AccelerationScene should be 1, but is %i", constraints.size());
        throw std::runtime_error("Invalid constraint configuration");
    }

    base::samples::RigidBodyStateSE3 ref_frame;

    // Create equation system
    //    Walk through all priorities and update the optimization problem. The outcome will be
    //    A - Vector of constraint matrices. One matrix for each priority
    //    y - Vector of constraint velocities. One vector for each priority
    //    W - Vector of constraint weights. One vector for each priority

    int prio = 0; // Only one priority is implemented here!
    constraints_prio[prio].resize(n_constraint_variables_per_prio[prio], robot_model->noOfJoints());

    // Walk through all tasks of current priority
    uint row_index = 0;
    for(uint i = 0; i < constraints[prio].size(); i++){

        ConstraintPtr constraint = constraints[prio][i];
        
        constraint->checkTimeout();
        constraint->update(robot_model);

        uint n_vars = constraint->config.nVariables();

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
        if(constraint->activation == 0){
           constraint->y_ref.setZero();
           constraint->y_ref_root.setZero();
        }

        // Insert constraints into equation system of current priority at the correct position. Note: Weights will be zero if activations
        // for this constraint is zero or if the constraint is in timeout
        constraints_prio[prio].Wy.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->timeout);
        constraints_prio[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = constraint->A;
        constraints_prio[prio].lower_y.segment(row_index, n_vars) = constraint->y_ref_root;
        constraints_prio[prio].upper_y.segment(row_index, n_vars) = constraint->y_ref_root;

        row_index += n_vars;
    }
    const base::MatrixXd& A = constraints_prio[prio].A;
    const base::VectorXd& y = constraints_prio[prio].lower_y;

    // Cost Function: x^T*H*x + x^T * g
    constraints_prio[prio].H = A.transpose()*A;
    constraints_prio[prio].g.setZero() = -(A.transpose()*y).transpose();
    constraints_prio[prio].upper_x.setConstant(1000);
    constraints_prio[prio].lower_x.setConstant(-1000);

    constraints_prio[prio].A.setZero();
    constraints_prio[prio].lower_y.setZero();
    constraints_prio[prio].upper_y.setZero();

    constraints_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all constraints!?
    constraints_prio.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    return constraints_prio;
}

const base::commands::Joints& AccelerationScene::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    // Convert Output
    solver_output_joints.resize(robot_model->noOfActuatedJoints());
    solver_output_joints.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        uint idx = robot_model->jointIndex(name);
        solver_output_joints[name].acceleration = solver_output[idx];
    }
    solver_output_joints.time = base::Time::now();
    return solver_output_joints;
}

const ConstraintsStatus &AccelerationScene::updateConstraintsStatus(){

    robot_acc.resize(robot_model->noOfJoints());
    uint nj = robot_model->noOfJoints();
    const base::samples::Joints &joint_state = robot_model->jointState(robot_model->jointNames());
    for(size_t i = 0; i < nj; i++)
        robot_acc(i) = joint_state[i].acceleration;

    for(uint prio = 0; prio < constraints.size(); prio++){
        for(uint i = 0; i < constraints[prio].size(); i++){
            ConstraintPtr constraint = constraints[prio][i];
            const std::string &name = constraint->config.name;

            constraints_status[name].time       = constraint->time;
            constraints_status[name].config     = constraint->config;
            constraints_status[name].activation = constraint->activation;
            constraints_status[name].timeout    = constraint->timeout;
            constraints_status[name].weights    = constraint->weights;
            constraints_status[name].y_ref      = constraint->y_ref_root;
            if(constraint->config.type == cart){
                const base::MatrixXd &jac = robot_model->spaceJacobian(constraint->config.root, constraint->config.tip);
                const base::Acceleration &bias_acc = robot_model->spatialAccelerationBias(constraint->config.root, constraint->config.tip);
                constraints_status[name].y_solution = jac * solver_output + bias_acc;
                constraints_status[name].y          = jac * robot_acc + bias_acc;
            }
        }
    }

    return constraints_status;
}

} // namespace wbc
