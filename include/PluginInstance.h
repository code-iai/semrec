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
    
    Result loadPluginLibrary(string strFilepath, int argc, char** argv);
    void unload();
    
    Result cycle();
    
    bool subscribedToEvent(EventIdentifier eiEventIdentifier);
    void consumeEvent(Event evEvent);
  };
}


#endif /* __PLUGIN_INSTANCE_H__ */
