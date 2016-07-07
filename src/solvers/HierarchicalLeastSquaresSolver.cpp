#include "HierarchicalLeastSquaresSolver.hpp"
#include <base/logging.h>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include "../common/OptProblem.hpp"
#include <stdexcept>

using namespace std;

namespace wbc{

HierarchicalLeastSquaresSolver::HierarchicalLeastSquaresSolver() :
    no_of_joints(0),
    configured(false),
    min_eigenvalue(1e-9),
    max_solver_output_norm(10){
}

bool HierarchicalLeastSquaresSolver::configure(const std::vector<int>& n_constraint_per_prio, const unsigned int n_joints){

    priorities.clear();

    if(n_joints == 0){
        LOG_ERROR("No of joint variables must be > 0");
        return false;
    }

    if(n_constraint_per_prio.size() == 0){
        LOG_ERROR("No of priority levels (size of n_constraint_per_prio) has to be > 0");
        return false;
    }

    for(uint i = 0; i < n_constraint_per_prio.size(); i++){
        if(n_constraint_per_prio[i] == 0){
            LOG_ERROR("No of constraint variables on each priority level must be > 0");
            return false;
        }
    }

    for(uint prio = 0; prio < n_constraint_per_prio.size(); prio++){
        priorities.push_back(PriorityData(n_constraint_per_prio[prio], n_joints));
    }

    no_of_joints = n_joints;
    proj_mat.resize(no_of_joints, no_of_joints);
    proj_mat.setIdentity();
    s_vals.resize(no_of_joints);
    s_vals.setZero();
    sing_vect_r.resize(no_of_joints, no_of_joints);
    sing_vect_r.setIdentity();
    s_vals_inv.resize( no_of_joints, no_of_joints);
    s_vals_inv.setZero();
    damped_s_vals_inv.resize( no_of_joints, no_of_joints);
    damped_s_vals_inv.setZero();
    Wq_V.resize(no_of_joints, no_of_joints);
    Wq_V.setZero();
    Wq_V_s_vals_inv.resize(no_of_joints, no_of_joints);
    Wq_V_s_vals_inv.setZero();
    Wq_V_damped_s_vals_inv.resize(no_of_joints, no_of_joints);
    Wq_V_damped_s_vals_inv.setZero();

    configured = true;

    tmp.resize(no_of_joints);
    tmp.setZero();

    max_solver_output.setConstant(no_of_joints, std::numeric_limits<double>::infinity());

    return true;
}

void HierarchicalLeastSquaresSolver::solve(const OptProblem &opt_problem, Eigen::VectorXd &solver_output){

    if(!configured)
        throw std::runtime_error("HierarchicalLeastSquaresSolver has to be configured before calling solve()!");

    HierarchicalWeightedLS& opt_problem_ls = (HierarchicalWeightedLS& )opt_problem;

    // Check valid input
    if(opt_problem_ls.priorities.size() != priorities.size()){
        LOG_ERROR("Number of priorities in solver: %i, Size of input vector: %i", priorities.size(), opt_problem_ls.priorities.size());
        throw std::invalid_argument("Invalid solver input");
    }

    solver_output.resize(no_of_joints);
    solver_output.setZero();

    // Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat.setIdentity();

    //////// Loop through all priorities

    for(uint prio = 0; prio < priorities.size(); prio++){

        const WeightedLS& opt_problem_prio = ((HierarchicalWeightedLS& )opt_problem).priorities[prio];

        if(opt_problem_prio.A.rows()     != priorities[prio].n_constraint_variables ||
           opt_problem_prio.A.cols()     != no_of_joints ||
           opt_problem_prio.y_ref.size() != priorities[prio].n_constraint_variables){
            LOG_ERROR("Expected input size on priority level %i: A: %i x %i, b: %i x 1, actual input: A: %i x %i, b: %i x 1",
                      prio, priorities[prio].n_constraint_variables, no_of_joints, priorities[prio].n_constraint_variables, opt_problem_prio.A.rows(), opt_problem_prio.A.cols(), opt_problem_prio.y_ref.size());
            throw std::invalid_argument("Invalid size of input variables");
        }

        // Set weights for this prioritiy
        setConstraintWeights(opt_problem_prio.W, prio);

        priorities[prio].y_comp.setZero();

        // Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities[prio].y_comp = opt_problem_prio.y_ref - opt_problem_prio.A*solver_output;

        // projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        // For the first priority P == Identity
        priorities[prio].A_proj = opt_problem_prio.A * proj_mat;

        // Compute weighted, projected mat: A_proj_w = Wy * A_proj * Wq^-1
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication

        for(uint i = 0; i < priorities[prio].n_constraint_variables; i++)
            priorities[prio].A_proj_w.row(i) = priorities[prio].constraint_weight_mat(i,i) * priorities[prio].A_proj.row(i);

        for(uint i = 0; i < no_of_joints; i++)
            priorities[prio].A_proj_w.col(i) = priorities[prio].joint_weight_mat(i,i) * priorities[prio].A_proj_w.col(i);

        KDL::svd_eigen_HH(priorities[prio].A_proj_w, priorities[prio].U, s_vals, sing_vect_r, tmp);

        // Compute damping factor based on
        // A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        // Robotic Manipulators through Kinematically Singular Configurations”,
        // Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double s_min = s_vals.block(0,0,min(no_of_joints, priorities[prio].n_constraint_variables),1).minCoeff();
        if(s_min <= (1/max_solver_output_norm)/2)
            priorities[prio].damping = (1/max_solver_output_norm)/2;
        else if(s_min >= (1/max_solver_output_norm))
            priorities[prio].damping = 0;
        else
            priorities[prio].damping = sqrt(s_min*((1/max_solver_output_norm)-s_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        damped_s_vals_inv.setZero();
        for (uint i = 0; i < min(no_of_joints, priorities[prio].n_constraint_variables); i++)
            damped_s_vals_inv(i,i) = (s_vals(i) / (s_vals(i) * s_vals(i) + priorities[prio].damping * priorities[prio].damping));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < s_vals.rows(); i++){
            if(s_vals(i) < min_eigenvalue)
                s_vals_inv(i,i) = 0;
            else
                s_vals_inv(i,i) = 1 / s_vals(i);
        }

        // A^# = Wq^-1 * V * S^# * U^T * Wy
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication (saves a lot of computation!)
        priorities[prio].u_t_weight_mat = priorities[prio].U.transpose();
        for(uint i = 0; i < priorities[prio].n_constraint_variables; i++)
            priorities[prio].u_t_weight_mat.col(i) = priorities[prio].u_t_weight_mat.col(i) * priorities[prio].constraint_weight_mat(i,i);

        for(uint i = 0; i < no_of_joints; i++)
            Wq_V.row(i) = priorities[prio].joint_weight_mat(i,i) * sing_vect_r.row(i);

        for(uint i = 0; i < no_of_joints; i++)
            Wq_V_s_vals_inv.col(i) = Wq_V.col(i) * s_vals_inv(i,i);
        for(uint i = 0; i < no_of_joints; i++)
            Wq_V_damped_s_vals_inv.col(i) = Wq_V.col(i) * damped_s_vals_inv(i,i);

        priorities[prio].A_proj_inv_wls = Wq_V_s_vals_inv * priorities[prio].u_t_weight_mat; //Normal Inverse with weighting
        priorities[prio].A_proj_inv_wdls = Wq_V_damped_s_vals_inv * priorities[prio].u_t_weight_mat; //Damped inverse with weighting

        // x = x + A^# * y
        priorities[prio].solution_prio = priorities[prio].A_proj_inv_wdls * priorities[prio].y_comp;
        solver_output += priorities[prio].solution_prio;

        // Compute projection matrix for the next priority. Use here the undamped inverse to have a correct solution
        proj_mat -= priorities[prio].A_proj_inv_wls * priorities[prio].A_proj;

        //store eigenvalues for this priority
        priorities[prio].sing_vals.setZero();
        for(uint i = 0; i < no_of_joints; i++)
            priorities[prio].sing_vals(i) = s_vals(i);

    } //priority loop

    // If a max solver output is given, scale all values according to the maximum allowed values
    applySaturation(solver_output);

    ///////////////
}

void HierarchicalLeastSquaresSolver::setJointWeights(const Eigen::VectorXd& weights, const uint prio){
    if(prio < 0 ||prio >= priorities.size()){
        LOG_ERROR("Cannot set joint weights on priority %i. Number of priority levels is %i", prio, priorities.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(weights.size() == no_of_joints){
        priorities[prio].joint_weight_mat.setZero();
        for(uint i = 0; i < no_of_joints; i++)
        {
            if(weights(i) >= 0)
                priorities[prio].joint_weight_mat(i,i) = sqrt(weights(i));
            else{
                LOG_ERROR("Entries of joint weight vector have to >= 0, but element %i is %f", i, weights(i));
                throw std::invalid_argument("Invalid Joint weight vector");
            }
        }
    }
    else{
        LOG_ERROR("Cannot set joint weights. Size of joint weight vector is %i but should be %i", weights.size(), no_of_joints);
        throw std::invalid_argument("Invalid Joint weight vector");
    }
}

void HierarchicalLeastSquaresSolver::setConstraintWeights(const Eigen::VectorXd& weights, const uint prio){
    if(prio < 0 ||prio >= priorities.size()){
        LOG_ERROR("Cannot set constraint weights on priority %i. Number of priority levels is %i", prio, priorities.size());
        throw std::invalid_argument("Invalid Priority");
    }
    if(priorities[prio].n_constraint_variables != weights.size()){
        LOG_ERROR("Cannot set constraint weights. Size of weight vector is %i but should be %i", weights.size(), priorities[prio].n_constraint_variables);
        throw std::invalid_argument("Invalid Weight vector size");
    }
    priorities[prio].constraint_weight_mat.setZero();
    for(uint i = 0; i < priorities[prio].n_constraint_variables; i++){
        if(weights(i) >= 0)
           priorities[prio].constraint_weight_mat(i,i) = sqrt(weights(i));
        else{
            LOG_ERROR("Entries of constraint weight vector have to >= 0, but element %i is %f", i, weights(i));
            throw std::invalid_argument("Invalid Constraint weight vector");
        }
    }
}

void HierarchicalLeastSquaresSolver::setMinEigenvalue(double _min_eigenvalue){
    if(_min_eigenvalue <= 0){
        throw std::invalid_argument("Min. Eigenvalue has to be > 0!");
    }
    min_eigenvalue = _min_eigenvalue;
}

void HierarchicalLeastSquaresSolver::setMaxSolverOutputNorm(double norm_max){
    if(norm_max <= 0){
        throw std::invalid_argument("Norm Max has to be > 0!");
    }
    max_solver_output_norm = norm_max;
}

void HierarchicalLeastSquaresSolver::setMaxSolverOutput(const Eigen::VectorXd& max_solver_output){
    if(max_solver_output.size() > 0 && max_solver_output.size() != no_of_joints)
        throw std::invalid_argument("Size of max solver output vector has to be same as number of joints");
    for(uint i = 0; i < max_solver_output.size(); i++){
        if(max_solver_output(i) <= 0)
            throw std::invalid_argument("All entries of max solver output have to > 0!");
    }
    this->max_solver_output = max_solver_output;
}
void HierarchicalLeastSquaresSolver::applySaturation(Eigen::VectorXd& solver_output){
    //Apply saturation. Scale all values according to the maximum output
    double scale = 1;
    for(uint i = 0; i < solver_output.size(); i++)
        scale = std::min( scale, max_solver_output(i)/fabs(solver_output(i)) );
    solver_output = scale * solver_output;
}
}
