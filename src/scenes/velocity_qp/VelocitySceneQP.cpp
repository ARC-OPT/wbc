#include "VelocitySceneQP.hpp"

#include "../../tools/Logger.hpp"

#include "../../constraints/ContactsVelocityConstraint.hpp"
#include "../../constraints/JointLimitsVelocityConstraint.hpp"


namespace wbc{

SceneRegistry<VelocitySceneQP> VelocitySceneQP::reg("velocity_qp");

VelocitySceneQP::VelocitySceneQP(RobotModelPtr robot_model, QPSolverPtr solver, const double dt) :
    Scene(robot_model, solver, dt),
    hessian_regularizer(1e-8),
    configured(false){

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    constraints.push_back(std::make_shared<ContactsVelocityConstraint>());
    constraints.push_back(std::make_shared<JointLimitsVelocityConstraint>(dt));
}

bool VelocitySceneQP::configure(const std::vector<TaskPtr> &tasks_in){
    if(tasks_in.empty()){
        log(logERROR)<<"Empty WBC Task configuration";
        return false;
    }
    for(const TaskPtr& task : tasks_in){
        if(task->type != TaskType::spatial_velocity &&
           task->type != TaskType::com_velocity &&
           task->type != TaskType::joint_velocity){
            log(logERROR)<<"Invalid task type: "<<task->type;
            return false;
        }
    }

    solver->reset();
    tasks.resize(1);
    tasks = tasks_in;
    hqp.resize(1);
    configured = true;

    return true;
}

const HierarchicalQP& VelocitySceneQP::update(){

    assert(configured);

    uint nj = robot_model->nj();
    uint prio = 0;

    ///////// Constraints

    // check problem size
    uint total_eqs = 0, total_ineqs = 0;
    bool has_bounds = false;
    for(auto constraint : constraints) {
        constraint->update(robot_model);
        if(constraint->type() == Constraint::equality)
            total_eqs += constraint->size();
        else if(constraint->type() == Constraint::inequality)
            total_ineqs += constraint->size();
        else if(constraint->type() == Constraint::bounds)
            has_bounds = true;
    }

    ///////// Tasks

    // QP Size: (ncp*6 x nj)
    // Variable order: (qd)
    QuadraticProgram &qp = hqp[0];
    qp.resize(nj, total_eqs, total_ineqs, has_bounds);
    qp.lower_y.setConstant(-99999);
    qp.upper_y.setConstant(+99999);
    qp.lower_x.setConstant(-99999);
    qp.upper_x.setConstant(+99999);
    qp.A.setZero();

    total_eqs = total_ineqs = 0;
    for(uint i = 0; i < constraints.size(); i++) {
        Constraint::Type type = constraints[i]->type();
        uint c_size = constraints[i]->size();

        if(type == Constraint::bounds) {
            qp.lower_x = constraints[i]->lb();
            qp.upper_x = constraints[i]->ub();
        }
        else if (type == Constraint::equality) {
            qp.A.middleRows(total_eqs, c_size) = constraints[i]->A();
            qp.b.segment(total_eqs, c_size) = constraints[i]->b();
            total_eqs += c_size;
        }
        else if (type == Constraint::inequality) {
            qp.C.middleRows(total_ineqs, c_size) = constraints[i]->A();
            qp.lower_y.segment(total_ineqs, c_size) = constraints[i]->lb();
            qp.upper_y.segment(total_ineqs, c_size) = constraints[i]->ub();
            total_ineqs += c_size;
        }
    }

    ///////// Tasks    
    qp.H.setZero();
    qp.g.setZero();
    hqp.Wq = Eigen::VectorXd::Map(robot_model->getJointWeights().data(), robot_model->nj());
    for(uint i = 0; i < tasks.size(); i++){
        
        TaskPtr task = tasks[i];
        task->update();

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0){
           task->y_ref.setZero();
           task->y_ref_world.setZero();
        }

        for(int i = 0; i < task->A.rows(); i++){
            task->Aw.row(i) = task->weights_world(i) * task->A.row(i) * task->activation;
            task->y_ref_world(i) = task->y_ref_world(i) * task->weights_world(i) * task->activation;
        }
        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = hqp.Wq[i] * task->Aw.col(i);

        qp.H.block(0,0,nj,nj) += task->Aw.transpose()*task->Aw;
        qp.g.segment(0,nj) -= task->Aw.transpose()*task->y_ref_world;

    } // tasks on prio

    // Add regularization term
    qp.H.block(0,0,nj,nj).diagonal().array() += hessian_regularizer;

    return hqp;
}

const types::JointCommand& VelocitySceneQP::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    // Convert Output
    uint nj = robot_model->nj();
    uint na = robot_model->na();
    uint nfb = robot_model->nfb();
    solver_output_joints.resize(na);

    for(uint i = 0; i < solver_output.size(); i++){
        if(std::isnan(solver_output[i]))
            throw std::runtime_error("Solver output is NaN");
    }

    solver_output_joints.velocity = solver_output.segment(nfb,nj-nfb);
    return solver_output_joints;
}

} // namespace wbc
