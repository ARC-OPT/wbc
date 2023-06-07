#ifndef QP_SOLVER_CONFIG_HPP
#define QP_SOLVER_CONFIG_HPP

#include <string>

namespace wbc {

struct QPSolverConfig{
public:
    QPSolverConfig() :
        type("qpoases"){
    }
    QPSolverConfig(const std::string type, const std::string file) :
        type(type),
        file(file){
    }
    void validate(){
        if(type.empty())
            throw std::runtime_error("Invalid solver config. Type must not be empty!");
    }
    std::string type;
    std::string file;
};

}

#endif
