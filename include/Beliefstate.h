#ifndef __BELIEFSTATE_H__
#define __BELIEFSTATE_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>
#include <libconfig.h++>

// ROS
#include <ros/package.h>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginSystem.h>

using namespace std;
using namespace libconfig;


namespace beliefstate {
  class Beliefstate {
  private:
    PluginSystem* m_psPlugins;
    bool m_bRun;
    int m_argc;
    char** m_argv;
    string m_strBaseDataDirectory;
    string m_strExperimentNameMask;
    list<string> m_lstPluginsToLoad;
    list<Event> m_lstGlobalEvents;
    
    // MongoDB specific settings
    bool m_bUseMongoDB;
    string m_strMongoDBHost;
    int m_nMongoDBPort;
    string m_strMongoDBDatabase;
    
  public:
    Beliefstate(int argc, char** argv);
    ~Beliefstate();
    
    Result init(string strConfigFile = "");
    Result deinit();
    
    bool loadConfigFile(string strConfigFile);
    bool fileExists(string strFileName);
    void replaceStringInPlace(string& subject, const string& search, const string& replace);
    
    void spreadEvent(Event evEvent);
    void spreadServiceEvent(ServiceEvent seServiceEvent);
    bool cycle();
    
    void triggerShutdown();
    
    void setBaseDataDirectory(string strBaseDataDirectory);
    string baseDataDirectory();
  };
}


#endif /* __BELIEFSTATE_H__ */
