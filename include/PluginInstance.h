#ifndef __PLUGIN_INSTANCE_H__
#define __PLUGIN_INSTANCE_H__


// System
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <dlfcn.h>
#include <thread>

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
    thread* m_thrdPluginCycle;
    bool m_bRunCycle;
    mutex m_mtxCycleResults;
    Result m_resCycleResult;
    
  public:
    PluginInstance();
    ~PluginInstance();
    
    Result loadPluginLibrary(string strFilepath);
    void unload();
    
    int pluginID();
    
    Result init(int argc, char** argv);
    Result cycle();
    void spinCycle();
    list<string> dependencies();
    
    bool subscribedToEvent(string strEventName);
    void consumeEvent(Event evEvent);
    bool offersService(string strServiceName);
    Event consumeServiceEvent(ServiceEvent seServiceEvent);
    
    string name();
    
    void setBaseDataDirectory(string strBaseDataDirectory);
    string baseDataDirectory();
    
    Result currentResult();
    void setRunning(bool bRunCycle);
    void waitForJoin();
  };
}


#endif /* __PLUGIN_INSTANCE_H__ */
