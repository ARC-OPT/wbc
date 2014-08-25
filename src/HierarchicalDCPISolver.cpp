#include "HierarchicalDCPISolver.hpp"
#include "SolverTypes.hpp"
#include <stdexcept>

namespace wbc{

HierarchicalDCPISolver::HierarchicalDCPISolver() :
    nx_(0)
{

}

HierarchicalDCPISolver::~HierarchicalDCPISolver()
{
    clearPriorities();
}

void HierarchicalDCPISolver::clearPriorities()
{
    for(uint i = 0; i < priorities_.size(); i++)
        delete priorities_[i];
    priorities_.clear();
}

bool HierarchicalDCPISolver::configure(const std::vector<uint> ny_per_prio, const uint nx)
{
    nx_ = nx;
    proj_.resize(nx, nx);
    proj_.setIdentity();

    clearPriorities();

    priorities_.resize(ny_per_prio.size());
    for(uint i = 0; i < ny_per_prio.size(); i++)
        priorities_[i] = new Priority(ny_per_prio[i], nx);

    return true;
}

void HierarchicalDCPISolver::solve(const SolverInput& input, Eigen::VectorXd& x)
{
    if(x.size() != nx_){
        std::stringstream ss;
        ss << "Size of input vector x should be " << nx_ << " but is " << x.size() << std::endl;
        throw std::invalid_argument(ss.str());
    }

    // Set projection operator to identity:
    // The highest priority can search for a solution in the whole configuration space
    proj_.setIdentity();

    // Walk through all priorities
    for(uint i = 0; i < input.priorities.size(); i++)
    {

    }
}

}
