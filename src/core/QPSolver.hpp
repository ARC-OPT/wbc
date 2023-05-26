#ifndef WBC_SOLVERS_SOLVER_HPP
#define WBC_SOLVERS_SOLVER_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <memory>
#include <map>
#include "QPSolverConfig.hpp"

namespace wbc{

class HierarchicalQP;

class QPSolver{
protected:
    bool configured;
public:
    QPSolver();
    virtual ~QPSolver();
    /**
     * @brief solve Solve the given quadratic program
     * @param hierarchical_qp Description of the hierarchical quadratic program to solve.
     * @param solver_output solution of the quadratic program
     */
    virtual void solve(const HierarchicalQP& hierarchical_qp, base::VectorXd &solver_output) = 0;

    /** @brief reset Enforces reconfiguration at next call to solve() */
    void reset(){configured=false;}
};

typedef std::shared_ptr<QPSolver> QPSolverPtr;

template<typename T> QPSolver* createT(){return new T;}

struct QPSolverFactory{
    typedef std::map<std::string, QPSolver*(*)()> QPSolverMap;

    static QPSolver *createInstance(const std::string& name) {
        QPSolverMap::iterator it = getQPSolverMap()->find(name);
        if(it == getQPSolverMap()->end())
            throw std::runtime_error("Failed to create instance of plugin " + name + ". Is the plugin registered?");
        return it->second();
    }

    template<typename T>
    static T* createInstance(const std::string& name){
        QPSolver* tmp = createInstance(name);
        T* ret = dynamic_cast<T*>(tmp);
        return ret;
    }

    static QPSolverMap *getQPSolverMap(){
        if(!qp_solver_map)
            qp_solver_map = new QPSolverMap;
        return qp_solver_map;
    }

    static void clear(){
        qp_solver_map->clear();
    }

private:
    static QPSolverMap *qp_solver_map;
};

template<typename T>
struct QPSolverRegistry : QPSolverFactory{
    QPSolverRegistry(const std::string& name) {
        QPSolverMap::iterator it = getQPSolverMap()->find(name);
        if(it != getQPSolverMap()->end())
            throw std::runtime_error("Failed to register plugin with name " + name + ". A plugin with the same name is already registered");
        getQPSolverMap()->insert(std::make_pair(name, &createT<T>));
    }
};
}

#endif
