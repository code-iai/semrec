#ifndef __PLUGIN_H__
#define __PLUGIN_H__


// System
#include <cstdlib>
#include <mutex>
#include <list>
#include <string>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class Plugin {
    private:
      list<string> m_lstDependencies;
      
    protected:
      list<Event> m_lstEvents;
      mutex m_mtxEventsStore;
      list<EventIdentifier> m_lstSubscribedEventIdentifiers;
      list<ServiceEvent> m_lstServiceEvents;
      mutex m_mtxServiceEventsStore;
      
    public:
      Plugin();
      ~Plugin();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      void setSubscribedToEvent(EventIdentifier eiEventIdentifier, bool bSubscribed);
      bool subscribedToEvent(EventIdentifier eiEventIdentifier);
      virtual void consumeEvent(Event evEvent);
      virtual void consumeServiceEvent(ServiceEvent seServiceEvent);
      
      void addDependency(string strPluginName);
      bool dependsOn(string strPluginName);
      list<string> dependencies();
      
      void deployCycleData(Result& resDeployTo);
    };
  }
}


#endif /* __PLUGIN_H__ */
