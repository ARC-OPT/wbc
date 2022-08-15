#include "VelocitySceneQuadraticCost.hpp"

#include <base/JointLimits.hpp>
#include <base-logging/Logging.hpp>

#include "../tasks/CartesianVelocityTask.hpp"
#include "../tasks/JointVelocityTask.hpp"
#include "../tasks/CoMVelocityTask.hpp"

#include "../constraints/ContactsVelocityConstraint.hpp"
#include "../constraints/JointLimitsVelocityConstraint.hpp"


namespace wbc{

VelocitySceneQuadraticCost::VelocitySceneQuadraticCost(RobotModelPtr robot_model, QPSolverPtr solver, double dt) :
    VelocityScene(robot_model, solver),
    hessian_regularizer(1e-8){

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    constraints.resize(1);
    constraints[0].push_back(std::make_shared<ContactsVelocityConstraint>());
    constraints[0].push_back(std::make_shared<JointLimitsVelocityConstraint>(dt));

}

VelocitySceneQuadraticCost::~VelocitySceneQuadraticCost(){
}

const HierarchicalQP& VelocitySceneQuadraticCost::update(){

    if(!configured)
        throw std::runtime_error("VelocitySceneQuadraticCost has not been configured!. Please call configure() before calling update() for the first time!");

    if(tasks.size() != 1){
        LOG_ERROR("Number of priorities in VelocitySceneQuadraticCost should be 1, but is %i", tasks.size());
        throw std::runtime_error("Invalid task configuration");
    }

    int nj = robot_model->noOfJoints();
    const ActiveContacts& contact_points = robot_model->getActiveContacts();
    uint ncp = contact_points.size();
    uint prio = 0;

    // QP Size: (NContacts*6 X NJoints)
    tasks_prio[prio].resize(ncp*6,nj);
    tasks_prio[prio].H.setZero();
    tasks_prio[prio].g.setZero();

    ///////// Tasks

    for(uint i = 0; i < tasks[prio].size(); i++){
        
        TaskPtr task = tasks[prio][i];

        task->checkTimeout();
        task->update(robot_model);

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0){
           task->y_ref.setZero();
           task->y_ref_root.setZero();
        }

        for(int i = 0; i < task->A.rows(); i++)
            task->Aw.row(i) = task->weights_root(i) * task->A.row(i) * task->activation * (!task->timeout);
        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = joint_weights[i] * task->Aw.col(i);

        tasks_prio[prio].H.block(0,0,nj,nj) += task->Aw.transpose()*task->Aw;
        tasks_prio[prio].g.segment(0,nj) -= task->Aw.transpose()*task->y_ref_root;

    } // tasks on prio

    // Add regularization term
    tasks_prio[prio].H.block(0,0,nj,nj).diagonal().array() += hessian_regularizer;


    ///////// Constraints

    size_t total_eqs = 0;
    for(auto constraint : constraints[prio]) {
        constraint->update(robot_model);
        if(constraint->type() != Constraint::bounds)
            total_eqs += constraint->size();
    }

    // Note already performed at the beginning of the update (but does not consider additional constriants)
    tasks_prio[prio].A.resize(total_eqs, nj);
    tasks_prio[prio].lower_x.resize(nj);
    tasks_prio[prio].upper_y.resize(nj);
    tasks_prio[prio].lower_y.resize(total_eqs);
    tasks_prio[prio].upper_y.resize(total_eqs);

    tasks_prio[prio].A.setZero();
    tasks_prio[prio].lower_y.setConstant(-99999);
    tasks_prio[prio].upper_y.setConstant(+99999);
    tasks_prio[prio].lower_x.setConstant(-99999);
    tasks_prio[prio].upper_x.setConstant(+99999);

    total_eqs = 0;
    for(uint i = 0; i < constraints[prio].size(); i++) {
        Constraint::Type type = constraints[prio][i]->type();
        size_t c_size = constraints[prio][i]->size();

        if(type == Constraint::bounds) {
            tasks_prio[prio].lower_x = constraints[prio][i]->lb();
            tasks_prio[prio].upper_x = constraints[prio][i]->ub();
        }
        else if (type == Constraint::equality) {
            tasks_prio[prio].A.middleRows(total_eqs, c_size) = constraints[prio][i]->A();
            tasks_prio[prio].lower_y.segment(total_eqs, c_size) = constraints[prio][i]->b();
            tasks_prio[prio].upper_y.segment(total_eqs, c_size) = constraints[prio][i]->b();
        }
        else if (type == Constraint::inequality) {
            tasks_prio[prio].A.middleRows(total_eqs, c_size) = constraints[prio][i]->A();
            tasks_prio[prio].lower_y.segment(total_eqs, c_size) = constraints[prio][i]->lb();
            tasks_prio[prio].upper_y.segment(total_eqs, c_size) = constraints[prio][i]->ub();
        }

        total_eqs += c_size;
    }

    tasks_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all tasks!?

    return tasks_prio;
}

} // namespace wbc
