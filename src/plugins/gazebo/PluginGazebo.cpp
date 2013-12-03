#include <plugins/gazebo/PluginGazebo.h>


namespace beliefstate {
  namespace plugins {
    PluginGazebo::PluginGazebo() {
      this->addDependency("ros");
    }
    
    PluginGazebo::~PluginGazebo() {
    }
    
    Result PluginGazebo::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setOffersService("spawn_model", true);
      
      return resInit;
    }
    
    Result PluginGazebo::deinit() {
      return defaultResult();
    }
    
    Result PluginGazebo::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginGazebo::consumeEvent(Event evEvent) {
      this->info("Consume event!");
    }
    
    Event PluginGazebo::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evService = defaultEvent();
      
      this->info("Consume service event of type '" + seServiceEvent.strServiceName + "'!");
      
      return evService;
    }
  }
  
  extern "C" plugins::PluginGazebo* createInstance() {
    return new plugins::PluginGazebo();
  }
  
  extern "C" void destroyInstance(plugins::PluginGazebo* icDestroy) {
    delete icDestroy;
  }
}
