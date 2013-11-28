#include <Beliefstate.h>


namespace beliefstate {
  Beliefstate::Beliefstate(int argc, char** argv) {
    m_psPlugins = NULL;
    m_bRun = true;
    m_argc = argc;
    m_argv = argv;
  }
  
  Beliefstate::~Beliefstate() {
    if(m_psPlugins) {
      delete m_psPlugins;
    }
  }
  
  Result Beliefstate::init() {
    Result resInit = defaultResult();
    
    // Do the actual init here.
    m_psPlugins = new PluginSystem(m_argc, m_argv);
    m_psPlugins->addPluginSearchPath("/home/winkler/groovy_overlay_ws/devel/lib/");

    m_psPlugins->loadPluginLibrary("symboliclog", true);
    m_psPlugins->loadPluginLibrary("gazebo", true);
    
    return resInit;
  }
  
  Result Beliefstate::deinit() {
    Result resInit = defaultResult();
    
    // Do the actual deinit here.
    
    return resInit;
  }
  
  void Beliefstate::spreadEvent(Event evEvent) {
    if(m_psPlugins->spreadEvent(evEvent) == 0) {
      cerr << "[beliefstate] Unhandled event dropped." << endl;
      
      if(evEvent.cdDesignator) {
	cerr << "[beliefstate] Content was:" << endl;
	evEvent.cdDesignator->printDesignator();
      } else {
	cerr << "[beliefstate] No content given." << endl;
      }
    }
  }
  
  void Beliefstate::spreadServiceEvent(ServiceEvent seServiceEvent) {
    if(m_psPlugins->spreadServiceEvent(seServiceEvent) == 0) {
      cerr << "[beliefstate] Unhandled service event ('" << seServiceEvent.strServiceName << "') dropped." << endl;
      
      if(seServiceEvent.cdDesignator) {
	cerr << "[beliefstate] Content was:" << endl;
	seServiceEvent.cdDesignator->printDesignator();
      } else {
	cerr << "[beliefstate] No content given." << endl;
      }
    }
  }
  
  bool Beliefstate::cycle() {
    bool bContinue = true;
    
    if(m_bRun) {
      Result resCycle = m_psPlugins->cycle();
      
      if(resCycle.bSuccess) {
	// Events
	for(list<Event>::iterator itEvent = resCycle.lstEvents.begin();
	    itEvent != resCycle.lstEvents.end();
	    itEvent++) {
	  Event evEvent = *itEvent;
	  
	  // Distribute the event
	  this->spreadEvent(evEvent);
	  
	  // Clean up
	  if(evEvent.cdDesignator) {
	    delete evEvent.cdDesignator;
	  }
	}
	
	// Services
	for(list<ServiceEvent>::iterator itEvent = resCycle.lstServiceEvents.begin();
	    itEvent != resCycle.lstServiceEvents.end();
	    itEvent++) {
	  ServiceEvent seServiceEvent = *itEvent;
	  
	  // Distribute the event
	  this->spreadServiceEvent(seServiceEvent);
	  
	  // Clean up
	  if(seServiceEvent.cdDesignator) {
	    delete seServiceEvent.cdDesignator;
	  }
	}
      }
    } else {
      bContinue = false;
    }
    
    return bContinue;
  }
  
  void Beliefstate::triggerShutdown() {
    m_bRun = false;
  }
}
