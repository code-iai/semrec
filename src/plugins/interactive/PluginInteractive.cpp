#include <plugins/interactive/PluginInteractive.h>


namespace beliefstate {
  namespace plugins {
    PluginInteractive::PluginInteractive() {
      this->addDependency("ros");
      
      m_imsServer = NULL;
    }
    
    PluginInteractive::~PluginInteractive() {
      if(m_imsServer) {
	delete m_imsServer;
      }
    }
    
    Result PluginInteractive::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_imsServer = new InteractiveMarkerServer("interactive", "", false);
      InteractiveObject* ioObject = new InteractiveObject();
      ioObject->insertIntoServer(m_imsServer);
      //this->setOffersService("spawn_model", true);
      
      return resInit;
    }
    
    Result PluginInteractive::deinit() {
      return defaultResult();
    }
    
    Result PluginInteractive::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginInteractive::consumeEvent(Event evEvent) {
      this->info("Consume event!");
    }
    
    Event PluginInteractive::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evService = defaultEvent();
      
      this->info("Consume service event of type '" + seServiceEvent.strServiceName + "'!");
      
      return evService;
    }
  }
  
  extern "C" plugins::PluginInteractive* createInstance() {
    return new plugins::PluginInteractive();
  }
  
  extern "C" void destroyInstance(plugins::PluginInteractive* icDestroy) {
    delete icDestroy;
  }
}
