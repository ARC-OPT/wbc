#ifndef SCENE_CONFIG_HPP
#define SCENE_CONFIG_HPP

#include <string>

namespace wbc {

struct SceneConfig{
public:
    SceneConfig() :
        type("velocity"){
    }
    SceneConfig(const std::string type, const std::string file) :
        type(type),
        file(file){
    }
    void validate(){
        if(type.empty())
            throw std::runtime_error("Invalid scene config. Type must not be empty!");
    }
    std::string type;
    std::string file;
};

}

#endif
