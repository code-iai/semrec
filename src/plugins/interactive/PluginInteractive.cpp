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
      m_imsServer = new InteractiveMarkerServer("interactive_beliefstate", "", false);
      
      // Subscribe to internal events
      this->setSubscribedToEvent("symbolic-add-object", true);
      this->setSubscribedToEvent("symbolic-remove-object", true);
      this->setSubscribedToEvent("symbolic-update-object-pose", true);
      this->setSubscribedToEvent("symbolic-set-interactive-object-menu", true);
      
      // Just for development: Add objects
      //InteractiveObject* ioNew = this->addInteractiveObject("object0");
      //ioNew->addMenuEntry("Pick up", "pickup");
      
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
    
    void PluginInteractive::removeInteractiveObject(string strName) {
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	if((*itIO)->name() == strName) {
	  m_lstInteractiveObjects.erase(itIO);
	  break;
	}
      }
    }
    
    Result PluginInteractive::deinit() {
      return defaultResult();
    }
    
    Result PluginInteractive::cycle() {
      Result resCycle = defaultResult();
      
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	list<InteractiveObjectCallbackResult> lstCBResults = (*itIO)->callbackResults();
	
	for(list<InteractiveObjectCallbackResult>::iterator itCBR = lstCBResults.begin();
	    itCBR != lstCBResults.end();
	    itCBR++) {
	  InteractiveObjectCallbackResult iocrResult = *itCBR;
	  
	  Event evCallback = defaultEvent("interactive-callback");
	  evCallback.cdDesignator = new CDesignator();
	  evCallback.cdDesignator->setType(ACTION);
	  evCallback.cdDesignator->setValue("object", iocrResult.strObject);
	  evCallback.cdDesignator->setValue("command", iocrResult.strCommand);
	  evCallback.cdDesignator->setValue("parameter", iocrResult.strParameter);
	  
	  this->deployEvent(evCallback);
	}
      }
      
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginInteractive::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-add-object") {
	if(evEvent.cdDesignator) {
	  string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    this->addInteractiveObject(strObjectName);
	  } else {
	    this->warn("No name given when adding interactive object!");
	  }
	} else {
	  this->warn("No designator given when adding interactive object!");
	}
      } else if(evEvent.strEventName == "symbolic-remove-object") {
	this->warn("Removing objects via events not yet implemented.");
      } else if(evEvent.strEventName == "symbolic-update-object-pose") {
	this->warn("Updating object poses via events not yet implemented.");
      } else if(evEvent.strEventName == "symbolic-set-interactive-object-menu") {
	this->warn("Updating object menu via events not yet implemented.");
      }
    }
  }
  
  extern "C" plugins::PluginInteractive* createInstance() {
    return new plugins::PluginInteractive();
  }
  
  extern "C" void destroyInstance(plugins::PluginInteractive* icDestroy) {
    delete icDestroy;
  }
}
