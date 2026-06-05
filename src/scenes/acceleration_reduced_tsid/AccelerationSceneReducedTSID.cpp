#include "AccelerationSceneReducedTSID.hpp"
#include "core/RobotModel.hpp"
#include "../../tools/Logger.hpp"

#include "../../constraints/RigidbodyDynamicsConstraint.hpp"
#include "../../constraints/ContactsAccelerationConstraint.hpp"
#include "../../constraints/JointLimitsAccelerationConstraint.hpp"
#include "../../constraints/EffortLimitsAccelerationConstraint.hpp"
#include "../../constraints/ContactsFrictionPointConstraint.hpp"
#include "../../constraints/ContactsFrictionSurfaceConstraint.hpp"

namespace wbc {

SceneRegistry<AccelerationSceneReducedTSID> AccelerationSceneReducedTSID::reg("acceleration_reduced_tsid");

AccelerationSceneReducedTSID::AccelerationSceneReducedTSID(RobotModelPtr robot_model, QPSolverPtr solver, const double dt, uint dim_contact) :
    Scene(robot_model, solver, dt),
    configured(false),
    dim_contact(dim_contact){

    // whether or not torques are removed  from the qp problem
    // this formulation includes torques !!!
    bool reduced = true; // DO NOT CHANGE

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    if(robot_model->hasFloatingBase())
        constraints.push_back(std::make_shared<RigidbodyDynamicsConstraint>(reduced,dim_contact));
    constraints.push_back(std::make_shared<ContactsAccelerationConstraint>(reduced, dim_contact));
    constraints.push_back(std::make_shared<JointLimitsAccelerationConstraint>(dt, reduced, dim_contact));
    constraints.push_back(std::make_shared<EffortLimitsAccelerationConstraint>(dim_contact));
    if(dim_contact == 3)
        constraints.push_back(std::make_shared<ContactsFrictionPointConstraint>(reduced));
    else
        constraints.push_back(std::make_shared<ContactsFrictionSurfaceConstraint>(reduced));
}

bool AccelerationSceneReducedTSID::configure(const std::vector<TaskPtr> &tasks_in){
    if(tasks_in.empty()){
        log(logERROR)<<"Empty WBC Task configuration";
        return false;
    }
    for(const TaskPtr& task : tasks_in){
        if(task->type != spatial_acceleration &&
           task->type != joint_acceleration &&
           task->type != com_acceleration &&
           task->type != contact_force &&
           task->type != contact_wrench ){
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

const HierarchicalQP& AccelerationSceneReducedTSID::update(){

    assert(configured);

    uint nj = robot_model->nj();
    uint ncp = robot_model->nc();

    //////// Constraints

    bool has_bounds = false;
    size_t total_eqs = 0, total_ineqs = 0;
    uint n_friction = 0;
    for(auto contraint : constraints) {
        contraint->update(robot_model);
        if(contraint->type() == Constraint::equality)
            total_eqs += contraint->size();
        if(contraint->type() == Constraint::inequality) {
            total_ineqs += contraint->size();
            n_friction = contraint->size(); // friction is the only/last inequality
        }
        if(contraint->type() == Constraint::bounds)
            has_bounds = true;
    }
    if(n_friction > 0) has_bounds = true; // slack variables need bounds

    // Variable order: (qdd, f_ext, s) where s >= 0 are soft friction cone slack variables
    QuadraticProgram& qp = hqp[0];
    qp.resize(nj+ncp*dim_contact+n_friction, total_eqs, total_ineqs, has_bounds);
    qp.A.setZero();
    qp.C.setZero(); // must zero explicitly since resize() fills with NaN
    qp.lower_x.setConstant(-10000);   // bounds
    qp.upper_x.setConstant(+10000);   // bounds
    if(n_friction > 0)
        qp.lower_x.tail(n_friction).setZero(); // s >= 0
    qp.lower_y.setZero(); // inequalities
    qp.upper_y.setZero(); // inequalities

    total_eqs = 0, total_ineqs = 0;
    uint friction_row_start = 0;
    for(uint i = 0; i < constraints.size(); i++) {
        Constraint::Type type = constraints[i]->type();
        size_t c_size = constraints[i]->size();

        if(type == Constraint::bounds) {
            qp.lower_x.head(nj+ncp*dim_contact) = constraints[i]->lb(); // NOTE! Good only if a single bound task is admitted
            qp.upper_x.head(nj+ncp*dim_contact) = constraints[i]->ub();
        }
        else if (type == Constraint::equality) {
            qp.A.middleRows(total_eqs, c_size).leftCols(nj+ncp*dim_contact) = constraints[i]->A();
            qp.b.segment(total_eqs, c_size) = constraints[i]->b();
            total_eqs += c_size;
        }
        else if (type == Constraint::inequality) {
            friction_row_start = total_ineqs;
            qp.C.middleRows(total_ineqs, c_size).leftCols(nj+ncp*dim_contact) = constraints[i]->A();
            qp.lower_y.segment(total_ineqs, c_size) = constraints[i]->lb();
            qp.upper_y.segment(total_ineqs, c_size) = constraints[i]->ub();
            total_ineqs += c_size;
        }
    }

    // Soft friction cone: extend friction rows with -I for slack s, so A_fric*f - s <= 0
    if(n_friction > 0)
        qp.C.block(friction_row_start, nj+ncp*dim_contact, n_friction, n_friction) =
            -Eigen::MatrixXd::Identity(n_friction, n_friction);

    ///////// Tasks

    qp.H.setZero();
    qp.g.setZero();
    uint wrench_idx = 0;
    hqp.Wq = Eigen::VectorXd::Map(robot_model->getJointWeights().data(), robot_model->nj());
    for(uint i = 0; i < tasks.size(); i++){
        
        TaskPtr task = tasks[i];
        task->update();

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0)
           task->y_ref.setZero();

        for(int i = 0; i < task->A.rows(); i++){
            task->Aw.row(i) = task->weights(i) * task->A.row(i) * task->activation;
            task->y_ref(i) = task->y_ref(i) * task->weights(i) * task->activation;
        }

        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = hqp.Wq[i] * task->Aw.col(i);

        // Decide on which output variables the tasks are to be mapped: qdd or f_ext
        if(task->type == TaskType::contact_force || task->type == TaskType::contact_wrench){ // f_ext
            qp.H.block(nj+wrench_idx*dim_contact,nj+wrench_idx*dim_contact,dim_contact,dim_contact) += task->Aw.transpose()*task->Aw;
            qp.g.segment(nj+wrench_idx*dim_contact,dim_contact) -= task->Aw.transpose()*task->y_ref;
            wrench_idx++;
        }
        else{ // qdd
            qp.H.block(0,0,nj,nj) += task->Aw.transpose()*task->Aw;
            qp.g.segment(0,nj) -= task->Aw.transpose()*task->y_ref;
        }
    }
    qp.H.diagonal().array() += hessian_regularizer;
    //qp.H.block(nj,nj, ncp*3, ncp*3).diagonal().array() += 1e-12;

    // Soft friction cone: penalize slack s with rho*||s||^2 to keep forces inside the cone
    if(n_friction > 0)
        qp.H.block(nj+ncp*dim_contact, nj+ncp*dim_contact, n_friction, n_friction)
            .diagonal().array() += friction_softening_weight;

    // Tangential force rate regularization: penalize changes in fx,fy,tx,ty,tz but NOT fz.
    // Smooths forces at the friction cone boundary without restricting normal force changes
    // during contact transitions (foot landing/takeoff).
    if(f_ext_prev.size() == (int)(ncp * dim_contact)) {
        for(uint c = 0; c < ncp; c++) {
            for(int j = 0; j < (int)dim_contact; j++) {
                if(j == 2) continue; // skip fz (normal force must change freely)
                int k = nj + c*dim_contact + j;
                qp.H(k, k) += force_rate_weight;
                qp.g(k)    -= force_rate_weight * f_ext_prev(c*dim_contact + j);
            }
        }
    }

    return hqp;
}

const types::JointCommand& AccelerationSceneReducedTSID::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    bool contacts_changed = contactsHaveChanged(contacts, robot_model->getContacts());
    if(contacts_changed)
        f_ext_prev.resize(0); // invalidate: no valid previous force to regularize against
    contacts = robot_model->getContacts();
    solver->solve(hqp, solver_output, !contacts_changed);

    // Convert solver output: Acceleration and torque
    uint nj = robot_model->nj();
    uint na = robot_model->na();
    uint nc = contacts.size();

    auto qdd_out = Eigen::Map<Eigen::VectorXd>(solver_output.data(), nj);
    auto fext_out = Eigen::Map<Eigen::VectorXd>(solver_output.data()+nj, dim_contact*nc);

    // computing torques from accelerations and forces (using last na equation from dynamic equations of motion)
    Eigen::VectorXd tau_out = robot_model->jointSpaceInertiaMatrix().bottomRows(na) * qdd_out;
    for(uint c = 0; c < nc; c++)
        tau_out += -robot_model->spaceJacobian(contacts[c].frame_id).topRows(dim_contact).transpose().bottomRows(na) * fext_out.segment(c*dim_contact,dim_contact);
    tau_out += robot_model->biasForces().bottomRows(na);

    for(uint i = 0; i < solver_output.size(); i++){
        if(std::isnan(solver_output[i])){
            throw std::runtime_error("Solver output is NaN");
        }
    }
    f_ext_prev = fext_out;
    solver_output_joints.acceleration = qdd_out.tail(na);
    solver_output_joints.effort = tau_out; // tau_out does not include fb dofs.

//    std::cerr << "Acc:   " << qdd_out.transpose() << std::endl;
//    std::cerr << "Tau:   " << tau_out.transpose() << std::endl;
//    std::cerr << "F_ext: " << fext_out.transpose() << std::endl << std::endl;

    // Convert solver output: contact wrenches
    contact_wrenches.resize(nc);
    for(uint i = 0; i < nc; i++){
        contact_wrenches[i].force = fext_out.segment(i*dim_contact, 3);
        if(dim_contact == 6)
            contact_wrenches[i].torque = fext_out.segment(i*dim_contact+3, 3);
        else
            contact_wrenches[i].torque.setZero();
    }

    return solver_output_joints;
}
}
