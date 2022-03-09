#ifndef PLUGIN_LOADER_HPP
#define PLUGIN_LOADER_HPP

#include <map>
#include <dlfcn.h>

namespace wbc {

class PluginLoader{
public:
    typedef std::map<std::string, void*> PluginMap;

    static void loadPlugin(const std::string& name){
        void *handle = dlopen(std::string(name).c_str(), RTLD_NOW);
        if(!handle)
            throw std::runtime_error("Failed to load plugin " + name);
        getPluginMap()->insert(std::make_pair(name, handle));
    }

    static void unloadPlugin(const std::string& name){
        PluginMap::iterator it = getPluginMap()->find(name);
        if(it == getPluginMap()->end())
            throw std::runtime_error("Failed to unload plugin " + name + ". Is the plugin loaded?");
        dlclose(it->second);
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

#endif
