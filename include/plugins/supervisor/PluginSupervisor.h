#ifndef __PLUGIN_SUPERVISOR_H__
#define __PLUGIN_SUPERVISOR_H__


// System
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginSupervisor : public Plugin {
    private:
      bool m_bFirstExperiment;
      
    public:
      PluginSupervisor();
      ~PluginSupervisor();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
    };
  }
  
  extern "C" plugins::PluginSupervisor* createInstance();
  extern "C" void destroyInstance(plugins::PluginSupervisor* icDestroy);
}


#endif /* __PLUGIN_SUPERVISOR_H__ */
