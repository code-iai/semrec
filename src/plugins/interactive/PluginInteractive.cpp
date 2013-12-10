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
      
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	delete *itIO;
      }
    }
    
    Result PluginInteractive::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Initialize server
      m_imsServer = new InteractiveMarkerServer("interactive", "", false);
      
      // Just for development: Add objects
      InteractiveObject* ioNew = this->addInteractiveObject(new InteractiveObject("object0"));
      ioNew->addMenuEntry("Pick up (left gripper)", "pickup", "left_gripper");
      ioNew->addMenuEntry("Pick up (right gripper)", "pickup", "right_gripper");
      
      return resInit;
    }
    
    InteractiveObject* PluginInteractive::addInteractiveObject(InteractiveObject* ioAdd) {
      m_lstInteractiveObjects.push_back(ioAdd);
      ioAdd->insertIntoServer(m_imsServer);
      
      return ioAdd;
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
  }
  
  extern "C" plugins::PluginInteractive* createInstance() {
    return new plugins::PluginInteractive();
  }
  
  extern "C" void destroyInstance(plugins::PluginInteractive* icDestroy) {
    delete icDestroy;
  }
}
