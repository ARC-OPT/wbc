#include "WbcVelocityScene.hpp"
#include "RobotModel.hpp"
#include "Solver.hpp"
#include "JointVelocityConstraint.hpp"
#include "CartesianVelocityConstraint.hpp"
#include "SVD.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

Constraint* WbcVelocityScene::createConstraint(const ConstraintConfig &config){

    if(config.type == cart)
        return new CartesianVelocityConstraint(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return new JointVelocityConstraint(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Constraint with name %s has an invalid constraint type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid constraint config");
    }
}

void WbcVelocityScene::solve(base::commands::Joints& ctrl_output){

    opt_problem.prios.resize(constraints.size());
    base::samples::RigidBodyState ref_frame, tip_in_root, root_in_base;

    // Create equation system
    //    Walk through all priorities and update the optimization problem. The outcome will be
    //    A - Vector of constraint matrices. One matrix for each priority
    //    y - Vector of constraint velocities. One vector for each priority
    //    W - Vector of constraint weights. One vector for each priority
    for(uint prio = 0; prio < constraints.size(); prio++){

        opt_problem.prios[prio].resize(n_constraint_variables_per_prio[prio], robot_model->noOfJoints());

        // Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraints[prio].size(); i++){


            constraints[prio][i]->checkTimeout();
            int type = constraints[prio][i]->config.type;
            uint n_vars = constraints[prio][i]->config.nVariables();

            if(type == cart){

                CartesianVelocityConstraint* constraint = (CartesianVelocityConstraint*)constraints[prio][i];

                // Create constraint jacobian
                tip_in_root = robot_model->rigidBodyState(constraint->config.root, constraint->config.tip);
                root_in_base = robot_model->rigidBodyState(robot_model->baseFrame(), constraint->config.root);
                constraint->jacobian.setIdentity();
                constraint->jacobian.changeRefPoint(-tip_in_root.position);
                constraint->jacobian.changeRefFrame(root_in_base.getTransform());

                //Invert constraint Jacobian
                svd_eigen_decomposition(constraint->jacobian, constraint->Uf, constraint->Sf, constraint->Vf, constraint->tmp);
                for (unsigned int j = 0; j < constraint->Sf.size(); j++){
                    if (constraint->Sf(j) > 0)
                        constraint->Uf.col(j) *= 1 / constraint->Sf(j);
                    else
                        constraint->Uf.col(j).setZero();
                }
                constraint->H = (constraint->Vf * constraint->Uf.transpose());

                // A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root:
                constraint->A = constraint->H.block(0, 0, n_vars, 6) * robot_model->jacobian(robot_model->baseFrame(), constraint->config.tip) -
                                constraint->H.block(0, 0, n_vars, 6) * robot_model->jacobian(robot_model->baseFrame(), constraint->config.root);

                // Convert input twist from the reference frame of the constraint to the base frame of the robot. We transform only the orientation of the
                // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
                // be the origin of ref frame, not the root frame. This is more intuitive when controlling the orientation of e.g. a robot' s end effector.
                ref_frame = robot_model->rigidBodyState(robot_model->baseFrame(), constraint->config.ref_frame);
                constraint->y_ref_root.segment(0,3) = ref_frame.orientation.toRotationMatrix() * constraint->y_ref.segment(0,3);
                constraint->y_ref_root.segment(3,3) = ref_frame.orientation.toRotationMatrix() * constraint->y_ref.segment(3,3);

                // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
                // assume positive values
                constraint->weights_root.segment(0,3) = ref_frame.orientation.toRotationMatrix() * constraint->weights.segment(0,3);
                constraint->weights_root.segment(3,3) = ref_frame.orientation.toRotationMatrix() * constraint->weights.segment(3,3);
                constraint->weights_root = constraint->weights_root.cwiseAbs();
            }
            else if(type == jnt){

                JointVelocityConstraint* constraint = (JointVelocityConstraint*)constraints[prio][i];

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

            Constraint* constraint = constraints[prio][i];

            // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
            // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
            if(constraint->activation == 0){
               constraint->y_ref.setZero();
               constraint->y_ref_root.setZero();
            }

            // Insert constraints into equation system of current priority at the correct position. Note: Weights will be zero if activations
            // for this constraint is zero or if the constraint is in timeout
            opt_problem.prios[prio].W.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->timeout);
            opt_problem.prios[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = constraint->A;
            opt_problem.prios[prio].y_ref.segment(row_index, n_vars) = constraint->y_ref_root;

            row_index += n_vars;

        } // constraints on prio
    } // priorities

    // Solve equation system
    solver->solve(opt_problem, solver_output);

    // Convert solver output
    ctrl_output.resize(robot_model->noOfJoints());
    ctrl_output.names = robot_model->jointNames();
    for(size_t i = 0; i < ctrl_output.size(); i++)
        ctrl_output[i].speed = solver_output(i);
    ctrl_output.time = base::Time::now();
}

void WbcVelocityScene::evaluateConstraints(const base::VectorXd& solver_output, const base::VectorXd& robot_vel){

    for(uint prio = 0; prio < constraints.size(); prio++){
        for(uint i = 0; i < constraints[prio].size(); i++){

            Constraint* constraint = constraints[prio][i];
            constraint->y_solution = constraint->A * solver_output;
            constraint->y = constraint->A * robot_vel;
        }
    }
}


} // namespace wbc
