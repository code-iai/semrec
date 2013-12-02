#ifndef __PLUGIN_INSTANCE_H__
#define __PLUGIN_INSTANCE_H__


// System
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <dlfcn.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>

using namespace std;


namespace beliefstate {
  class PluginInstance {
  private:
    void* m_vdLibHandle;
    plugins::Plugin* m_piInstance;
    string m_strName;
    
  public:
    PluginInstance();
    ~PluginInstance();
    
    Result loadPluginLibrary(string strFilepath);
    void unload();
    
    int pluginID();
    
    Result init(int argc, char** argv);
    Result cycle();
    list<string> dependencies();
    
    bool subscribedToEvent(string strEventName);
    void consumeEvent(Event evEvent);
    bool offersService(string strServiceName);
    Event consumeServiceEvent(ServiceEvent seServiceEvent);
    
    string name();
    
    void setBaseDataDirectory(string strBaseDataDirectory);
    string baseDataDirectory();
  };
}


#endif /* __PLUGIN_INSTANCE_H__ */
