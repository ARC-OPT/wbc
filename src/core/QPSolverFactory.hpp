#ifndef QPSOLVER_FACTORY_HPP
#define QPSOLVER_FACTORY_HPP

#include "QPSolver.hpp"
#include <map>

namespace wbc {

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
