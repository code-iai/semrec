#ifndef __BELIEFSTATE_H__
#define __BELIEFSTATE_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginSystem.h>

using namespace std;


namespace beliefstate {
  class Beliefstate {
  private:
    PluginSystem* m_psPlugins;
    bool m_bRun;
    int m_argc;
    char** m_argv;
    string m_strBaseDataDirectory;
    
  public:
    Beliefstate(int argc, char** argv);
    ~Beliefstate();
    
    Result init();
    Result deinit();
    
    void spreadEvent(Event evEvent);
    void spreadServiceEvent(ServiceEvent seServiceEvent);
    bool cycle();
    
    void triggerShutdown();
    
    void setBaseDataDirectory(string strBaseDataDirectory);
    string baseDataDirectory();
  };
}


#endif /* __BELIEFSTATE_H__ */
