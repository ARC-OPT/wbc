#include "WbcVelocityScene.hpp"
#include "../core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

ConstraintPtr WbcVelocityScene::createConstraint(const ConstraintConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianVelocityConstraint>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointVelocityConstraint>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Constraint with name %s has an invalid constraint type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid constraint config");
    }
}

void WbcVelocityScene::update(){

    constraints_prio.resize(constraints.size());
    CartesianState ref_frame;

    // Create equation system
    //    Walk through all priorities and update the optimization problem. The outcome will be
    //    A - Vector of constraint matrices. One matrix for each priority
    //    y - Vector of constraint velocities. One vector for each priority
    //    W - Vector of constraint weights. One vector for each priority
    for(uint prio = 0; prio < constraints.size(); prio++){

        constraints_prio[prio].resize(n_constraint_variables_per_prio[prio], robot_model->noOfJoints());

        // Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraints[prio].size(); i++){


            constraints[prio][i]->checkTimeout();
            int type = constraints[prio][i]->config.type;
            uint n_vars = constraints[prio][i]->config.nVariables();

            if(type == cart){

                CartesianVelocityConstraintPtr constraint = std::static_pointer_cast<CartesianVelocityConstraint>(constraints[prio][i]);

                // Constraint Jacobian
                constraint->A = robot_model->jacobian(constraint->config.root, constraint->config.tip);

                // Constraint reference
                // Convert input twist from the reference frame of the constraint to the base frame of the robot. We transform only the orientation of the
                // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
                // be the origin of ref frame, not the root frame. This is more intuitive when controlling the orientation of e.g. a robot' s end effector.
                ref_frame = robot_model->cartesianState(constraint->config.root, constraint->config.ref_frame);
                constraint->y_ref_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(0,3);
                constraint->y_ref_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(3,3);

                // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
                // assume positive values
                constraint->weights_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(0,3);
                constraint->weights_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(3,3);
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
            constraints_prio[prio].Wy.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->timeout);
            constraints_prio[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = constraint->A;
            constraints_prio[prio].lower_y.segment(row_index, n_vars) = constraint->y_ref_root;
            constraints_prio[prio].upper_y.segment(row_index, n_vars) = constraint->y_ref_root;
            constraints_prio[prio].lower_x.resize(0);
            constraints_prio[prio].upper_x.resize(0);
            constraints_prio[prio].H.setIdentity(robot_model->noOfJoints(), robot_model->noOfJoints());
            constraints_prio[prio].g.setZero();

            row_index += n_vars;

        } // constraints on prio
    } // priorities
}

const ConstraintsStatus& WbcVelocityScene::updateConstraintsStatus(const base::commands::Joints& solver_output, const base::samples::Joints& joint_state){

    if(solver_output.size() != robot_model->noOfJoints())
        throw std::runtime_error("Size of solver output is " + std::to_string(solver_output.size())
                                 + " but number of robot joints is " + std::to_string(robot_model->noOfJoints()));

    solver_output_vel.resize(solver_output.size());
    robot_vel.resize(joint_state.size());
    for(size_t i = 0; i < robot_model->noOfJoints(); i++)
        solver_output_vel(i) = solver_output[i].speed;
    const std::vector<std::string> joint_names = robot_model->jointNames();
    for(size_t i = 0; i < joint_names.size(); i++)
        robot_vel(i) = joint_state[joint_names[i]].speed;

    for(uint prio = 0; prio < constraints.size(); prio++){
        for(uint i = 0; i < constraints[prio].size(); i++){
            ConstraintPtr constraint = constraints[prio][i];
            const std::string &name = constraint->config.name;

            constraints_status[name].time       = constraint->time;
            constraints_status[name].config     = constraint->config;
            constraints_status[name].activation = constraint->activation;
            constraints_status[name].timeout    = constraint->timeout;
            constraints_status[name].weights    = constraint->weights;
            constraints_status[name].y_ref      = constraint->y_ref;
            constraints_status[name].y_solution = constraint->A * solver_output_vel;
            constraints_status[name].y          = constraint->A * robot_vel;
        }
    }

    return constraints_status;
}


} // namespace wbc
