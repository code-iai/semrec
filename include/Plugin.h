#ifndef __PLUGIN_H__
#define __PLUGIN_H__


// System
#include <cstdlib>
#include <mutex>
#include <list>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class Plugin {
    private:
    protected:
      list<Event> m_lstEvents;
      mutex m_mtxEventsStore;
      list<EventIdentifier> m_lstSubscribedEventIdentifiers;
      
    public:
      Plugin();
      ~Plugin();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      void setSubscribedToEvent(EventIdentifier eiEventIdentifier, bool bSubscribed);
      bool subscribedToEvent(EventIdentifier eiEventIdentifier);
      virtual void consumeEvent(Event evEvent);
    };
  }
}


#endif /* __PLUGIN_H__ */
