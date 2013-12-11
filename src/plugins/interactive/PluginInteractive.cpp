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
      InteractiveObject* ioNew = this->addInteractiveObject("object0");
      ioNew->addMenuEntry("Pick up", "pickup");
      ioNew->removeMenuEntry("pickup");
      
      return resInit;
    }
    
    InteractiveObject* PluginInteractive::updatePoseForInteractiveObject(string strName, geometry_msgs::Pose posUpdate) {
      InteractiveObject* ioUpdate = this->interactiveObjectForName(strName);
      
      if(!ioUpdate) {
	ioUpdate = this->addInteractiveObject(strName);
      }
      
      ioUpdate->setPose(posUpdate);
      
      return ioUpdate;
    }
    
    InteractiveObject* PluginInteractive::addInteractiveObject(string strName) {
      return this->addInteractiveObject(new InteractiveObject(strName));
    }
    
    InteractiveObject* PluginInteractive::addInteractiveObject(InteractiveObject* ioAdd) {
      m_lstInteractiveObjects.push_back(ioAdd);
      ioAdd->insertIntoServer(m_imsServer);
      
      return ioAdd;
    }
    
    InteractiveObject* PluginInteractive::interactiveObjectForName(string strName) {
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	InteractiveObject* ioCurrent = *itIO;
	
	if(ioCurrent->name() == strName) {
	  return ioCurrent;
	}
      }
      
      return NULL;
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
