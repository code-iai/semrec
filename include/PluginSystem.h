#ifndef __PLUGIN_SYSTEM_H__
#define __PLUGIN_SYSTEM_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginInstance.h>
#include <UtilityBase.h>

using namespace std;


namespace beliefstate {
  class PluginSystem : public UtilityBase {
  private:
    list<PluginInstance*> m_lstLoadedPlugins;
    list<PluginInstance*> m_lstUnloadPlugins;
    list<string> m_lstPluginSearchPaths;
    int m_argc;
    char** m_argv;
    bool m_bLoadDevelopmentPlugins;
    
  public:
    PluginSystem(int argc, char** argv);
    ~PluginSystem();
    
    void setLoadDevelopmentPlugins(bool bLoadDevelopmentPlugins);
    bool loadDevelopmentPlugins();
    
    string pluginNameFromPath(string strPath);
    bool pluginLoaded(string strPluginName);
    Result loadPluginLibrary(string strFilepath, bool bIsNameOnly = false);
    void queueUnloadPluginInstance(PluginInstance* icUnload);
    
    int spreadEvent(Event evEvent);
    int spreadServiceEvent(ServiceEvent seServiceEvent);
    Result cycle();
    
    void addPluginSearchPath(string strPath);
    
    PluginInstance* pluginInstanceByID(int nID);
  };
}


#endif /* __PLUGIN_SYSTEM_H__ */
