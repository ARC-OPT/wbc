#ifndef ROBOT_MODEL_FACTORY_HPP
#define ROBOT_MODEL_FACTORY_HPP

#include "RobotModel.hpp"
#include <map>

namespace wbc {

template<typename T> RobotModel* createT(){return new T;}

struct RobotModelFactory{
    typedef std::map<std::string, RobotModel*(*)()> RobotModelMap;

    static RobotModel *createInstance(const std::string& name) {
        RobotModelMap::iterator it = getRobotModelMap()->find(name);
        if(it == getRobotModelMap()->end())
            throw std::runtime_error("Failed to create instance of plugin " + name + ". Is the plugin registered?");
        return it->second();
    }

    template<typename T>
    static T* createInstance(const std::string& name){
        RobotModel* tmp = createInstance(name);
        T* ret = dynamic_cast<T*>(tmp);
        return ret;
    }

    static RobotModelMap *getRobotModelMap(){
        if(!robot_model_map)
            robot_model_map = new RobotModelMap;
        return robot_model_map;
    }
private:
    static RobotModelMap *robot_model_map;
};

template<typename T>
struct RobotModelRegistry : RobotModelFactory{
    RobotModelRegistry(const std::string& name) {
        RobotModelMap::iterator it = getRobotModelMap()->find(name);
        if(it != getRobotModelMap()->end())
            throw std::runtime_error("Failed to register plugin with name " + name + ". A plugin with the same name is already registered");
        getRobotModelMap()->insert(std::make_pair(name, &createT<T>));
    }
};
}

#endif
