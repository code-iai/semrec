#include <plugins/supervisor/PluginSupervisor.h>


namespace beliefstate {
  namespace plugins {
    PluginSupervisor::PluginSupervisor() {
    }
    
    PluginSupervisor::~PluginSupervisor() {
    }
    
    Result PluginSupervisor::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent("start-new-experiment", true);
      
      return resInit;
    }
    
    Result PluginSupervisor::deinit() {
      return defaultResult();
    }
    
    Result PluginSupervisor::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginSupervisor::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "start-new-experiment") {
	this->warn("Start new experiment not yet implemented.");
      }
    }
  }
  
  extern "C" plugins::PluginSupervisor* createInstance() {
    return new plugins::PluginSupervisor();
  }
  
  extern "C" void destroyInstance(plugins::PluginSupervisor* icDestroy) {
    delete icDestroy;
  }
}
