#ifndef __PLUGIN_H__
#define __PLUGIN_H__


// System
#include <cstdlib>
#include <mutex>
#include <list>
#include <string>
#include <sstream>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class Plugin {
    private:
      list<string> m_lstDependencies;
      string m_strName;
      int m_nID;
      bool m_bRunCycle;
      mutex m_mtxRunCycle;
      
    protected:
      list<Event> m_lstEvents;
      mutex m_mtxEventsStore;
      list<string> m_lstSubscribedEventNames;
      list<ServiceEvent> m_lstServiceEvents;
      mutex m_mtxServiceEventsStore;
      list<string> m_lstOfferedServices;
      list<int> m_lstOpenRequestIDs;
      
    public:
      Plugin();
      ~Plugin();
      
      void setPluginID(int nID);
      int pluginID();
      
      void setPluginName(string strName);
      string pluginName();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      Result cycleResults();
      
      void setSubscribedToEvent(string strEventName, bool bSubscribed);
      bool subscribedToEvent(string strEventName);
      virtual void consumeEvent(Event evEvent);
      
      void setOffersService(string strServiceName, bool bOffering);
      bool offersService(string strServiceName);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      void addDependency(string strPluginName);
      bool dependsOn(string strPluginName);
      list<string> dependencies();
      
      void deployCycleData(Result& resDeployTo);
      
      void deployEvent(Event evDeploy, bool bWaitForEvent = false);
      void deployServiceEvent(ServiceEvent seDeploy);
      
      string pluginIdentifierString(bool bBold);
      void warn(string strMessage);
      void info(string strMessage);
      
      int getTimeStamp();
      
      int openNewRequestID();
      bool isRequestIDOpen(int nID);
      void closeRequestID(int nID);
      bool isAnyRequestIDOpen();
      
      void setRunning(bool bRunCycle);
      bool running();
      
      void waitForEvent(Event evWait);
    };
  }
}


#endif /* __PLUGIN_H__ */
