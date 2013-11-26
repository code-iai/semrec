#include <plugins/symboliclog/PluginSymbolicLog.h>


namespace beliefstate {
  namespace plugins {
    PluginSymbolicLog::PluginSymbolicLog() {
    }
    
    PluginSymbolicLog::~PluginSymbolicLog() {
    }
    
    Result PluginSymbolicLog::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Plan node control events
      this->setSubscribedToEvent(EI_BEGIN_CONTEXT, true);
      this->setSubscribedToEvent(EI_END_CONTEXT, true);
      
      // Extra information assertion
      this->setSubscribedToEvent(EI_ADD_FAILURE, true);
      this->setSubscribedToEvent(EI_ADD_OBJECT, true);
      this->setSubscribedToEvent(EI_ADD_DESIGNATOR, true);
      this->setSubscribedToEvent(EI_ADD_IMAGE_FROM_FILE, true);
      
      return resInit;
    }
    
    Result PluginSymbolicLog::deinit() {
      return defaultResult();
    }
    
    Result PluginSymbolicLog::cycle() {
      Result resCycle = defaultResult();
      
      return resCycle;
    }
    
    void PluginSymbolicLog::consumeEvent(Event evEvent) {
      cout << "PluginSymbolicLog: Consume event!" << endl;
    }
  }
  
  extern "C" plugins::PluginSymbolicLog* createInstance() {
    return new plugins::PluginSymbolicLog();
  }
  
  extern "C" void destroyInstance(plugins::PluginSymbolicLog* icDestroy) {
    delete icDestroy;
  }
}
