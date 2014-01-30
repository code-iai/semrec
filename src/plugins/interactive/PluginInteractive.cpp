#include <plugins/interactive/PluginInteractive.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->addDependency("ros");
      this->setDevelopmentPlugin(true);
      
      m_imsServer = NULL;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_imsServer) {
	delete m_imsServer;
      }
      
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	delete *itIO;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
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
    
    InteractiveObject* PLUGIN_CLASS::updatePoseForInteractiveObject(string strName, geometry_msgs::Pose posUpdate) {
      InteractiveObject* ioUpdate = this->interactiveObjectForName(strName);
      
      if(!ioUpdate) {
	ioUpdate = this->addInteractiveObject(strName);
      }
      
      ioUpdate->setPose(posUpdate);
      
      return ioUpdate;
    }
    
    InteractiveObject* PLUGIN_CLASS::addInteractiveObject(string strName) {
      return this->addInteractiveObject(new InteractiveObject(strName));
    }
    
    InteractiveObject* PLUGIN_CLASS::addInteractiveObject(InteractiveObject* ioAdd) {
      m_lstInteractiveObjects.push_back(ioAdd);
      ioAdd->insertIntoServer(m_imsServer);
      
      return ioAdd;
    }
    
    InteractiveObject* PLUGIN_CLASS::interactiveObjectForName(string strName) {
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
    
    bool PLUGIN_CLASS::removeInteractiveObject(string strName) {
      for(list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	if((*itIO)->name() == strName) {
	  (*itIO)->removeFromServer();
	  m_lstInteractiveObjects.erase(itIO);
	  
	  return true;
	}
      }
      
      return false;
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
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
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-add-object") {
	if(evEvent.cdDesignator) {
	  string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    InteractiveObject* ioNew = this->addInteractiveObject(strObjectName);
	    ioNew->setPose(evEvent.cdDesignator->poseValue("pose"));
	    ioNew->setSize(evEvent.cdDesignator->floatValue("width"),
			   evEvent.cdDesignator->floatValue("depth"),
			   evEvent.cdDesignator->floatValue("height"));
	    
	    ioNew->clearMenuEntries();
	    
	    CKeyValuePair* ckvpMenu = evEvent.cdDesignator->childForKey("menu");
	    list<string> lstMenuKeys = ckvpMenu->keys();
	    
	    for(list<string>::iterator itKey = lstMenuKeys.begin();
		itKey != lstMenuKeys.end();
		itKey++) {
	      string strKey = *itKey;
	      
	      CKeyValuePair* ckvpMenuEntry = ckvpMenu->childForKey(strKey);
	      string strLabel = ckvpMenuEntry->stringValue("label");
	      string strParameter = ckvpMenuEntry->stringValue("parameter");
	      
	      ioNew->addMenuEntry(strLabel, strKey, strParameter);
	      this->info("Added menu entry '" + strKey + "': '" + strLabel + "'");
	    }
	    
	    this->info("Registered interactive object '" + strObjectName + "'.");
	  } else {
	    this->warn("No name given when adding interactive object!");
	  }
	} else {
	  this->warn("No designator given when adding interactive object!");
	}
      } else if(evEvent.strEventName == "symbolic-remove-object") {
	if(evEvent.cdDesignator) {
	  string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    if(!this->removeInteractiveObject(strObjectName)) {
	      this->warn("Tried to unregister non-existing interactive object '" + strObjectName + "'.");
	    }
	  }
	}
      } else if(evEvent.strEventName == "symbolic-update-object-pose") {
	if(evEvent.cdDesignator) {
	  string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    geometry_msgs::Pose psSet = evEvent.cdDesignator->poseValue("pose");
	    
	    this->updatePoseForInteractiveObject(strObjectName, psSet);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-interactive-object-menu") {
	if(evEvent.cdDesignator) {
	  string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    InteractiveObject* ioNew = this->interactiveObjectForName(strObjectName);
	    
	    if(ioNew) {
	      ioNew->clearMenuEntries();
	    
	      CKeyValuePair* ckvpMenu = evEvent.cdDesignator->childForKey("menu");
	      list<string> lstMenuKeys = ckvpMenu->keys();
	    
	      for(list<string>::iterator itKey = lstMenuKeys.begin();
		  itKey != lstMenuKeys.end();
		  itKey++) {
		string strKey = *itKey;
	      
		CKeyValuePair* ckvpMenuEntry = ckvpMenu->childForKey(strKey);
		string strLabel = ckvpMenuEntry->stringValue("label");
		string strParameter = ckvpMenuEntry->stringValue("parameter");
	      
		ioNew->addMenuEntry(strLabel, strKey, strParameter);
		this->info("Added menu entry '" + strKey + "': '" + strLabel + "'");
	      }
	    } else {
	      this->warn("Interactive object '" + strObjectName + "' not known when setting menu!");
	    }
	  } else {
	    this->warn("No name given when setting interactive object menu!");
	  }
	} else {
	  this->warn("No designator given when setting interactive object menu!");
	}
      }
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
