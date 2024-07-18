#ifndef WBC_CORE_PLUGIN_LOADER_HPP
#define WBC_CORE_PLUGIN_LOADER_HPP

#include <string>
#include <stdexcept>
#include <map>
#include <dlfcn.h>

namespace wbc {

/**
 * @brief Helper class to load the robot model, solver and scene plugins
 */
class PluginLoader{
public:
    typedef std::map<std::string, void*> PluginMap;

    static bool loadPlugin(const std::string& name){
        void *handle = dlopen(std::string(name).c_str(), RTLD_NOW);
        if(!handle)
            return false;
        getPluginMap()->insert(std::make_pair(name, handle));
        return true;
    }

    static bool unloadPlugin(const std::string& name){
        PluginMap::iterator it = getPluginMap()->find(name);
        if(it == getPluginMap()->end())
            return false;
        dlclose(it->second);
        plugin_map->erase(name);
        return true;
    }

    static PluginMap *getPluginMap(){
        if(!plugin_map)
            plugin_map = new PluginMap;
        return plugin_map;
    }
private:
    static PluginMap* plugin_map;
};

}

#endif // WBC_CORE_PLUGIN_LOADER_HPP
