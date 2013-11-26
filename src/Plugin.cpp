#include <Plugin.h>


namespace beliefstate {
  namespace plugins {
    Plugin::Plugin() {
      // Dummy.
    }
    
    Plugin::~Plugin() {
      // Dummy.
    }
    
    Result Plugin::init(int argc, char** argv) {
      // Dummy.
      return defaultResult();
    }
    
    Result Plugin::deinit() {
      // Dummy.
      return defaultResult();
    }
    
    Result Plugin::cycle() {
      // Dummy.
      return defaultResult();
    }
    
    void Plugin::setSubscribedToEvent(EventIdentifier eiEventIdentifier, bool bSubscribed) {
      m_lstSubscribedEventIdentifiers.remove(eiEventIdentifier);
      
      if(bSubscribed) {
	m_lstSubscribedEventIdentifiers.push_back(eiEventIdentifier);
      }
    }
    
    bool Plugin::subscribedToEvent(EventIdentifier eiEventIdentifier) {
      for(list<EventIdentifier>::iterator itEI = m_lstSubscribedEventIdentifiers.begin();
	  itEI != m_lstSubscribedEventIdentifiers.end();
	  itEI++) {
	if(*itEI == eiEventIdentifier) {
	  return true;
	}
      }
      
      return false;
    }
    
    void Plugin::consumeEvent(Event evEvent) {
      // Dummy.
    }
  }
}
