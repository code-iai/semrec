#ifndef __PLUGIN_SUPERVISOR_H__
#define __PLUGIN_SUPERVISOR_H__


#define PLUGIN_CLASS PluginSupervisor


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
    class PLUGIN_CLASS : public Plugin {
    private:
      bool m_bFirstExperiment;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_SUPERVISOR_H__ */
