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
    m_psPlugins->loadPluginLibrary("/home/winkler/groovy_overlay_ws/devel/lib/libbs_plugin_ros.so");
    m_psPlugins->loadPluginLibrary("/home/winkler/groovy_overlay_ws/devel/lib/libbs_plugin_imagecapturer.so");
    m_psPlugins->loadPluginLibrary("/home/winkler/groovy_overlay_ws/devel/lib/libbs_plugin_symboliclog.so");
    
    return resInit;
  }
  
  Result Beliefstate::deinit() {
    Result resInit = defaultResult();
    
    // Do the actual deinit here.
    
    return resInit;
  }
  
  void Beliefstate::spreadEvent(Event evEvent) {
    m_psPlugins->spreadEvent(evEvent);
  }
  
  bool Beliefstate::cycle() {
    bool bContinue = true;
    
    if(m_bRun) {
      Result resCycle = m_psPlugins->cycle();
      
      if(resCycle.bSuccess) {
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
