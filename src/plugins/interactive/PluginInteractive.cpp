#include <plugins/interactive/PluginInteractive.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->addDependency("ros");
      this->setPluginVersion("0.4");
      
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
	  
	  this->info("Interactive callback for object '" + iocrResult.strObject + "': '" + iocrResult.strCommand + "', '" + iocrResult.strParameter + "'");
	  
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
	    
	    bool bPoseGiven = false;
	    if(evEvent.cdDesignator->childForKey("pose") != NULL) {
	      ioNew->setPose(evEvent.cdDesignator->poseValue("pose"));
	      bPoseGiven = true;
	    }
	    
	    if(evEvent.cdDesignator->childForKey("pose-stamped") != NULL) {
	      geometry_msgs::PoseStamped psPose = evEvent.cdDesignator->poseStampedValue("pose-stamped");
	      ioNew->setPose(psPose.header.frame_id, psPose.pose);
	      bPoseGiven = true;
	    }
	    
	    if(!bPoseGiven) {
	      this->info("No pose given for object '" + strObjectName + "'.");
	    }
	    
	    float fWidth = 0.1;
	    float fDepth = 0.1;
	    float fHeight = 0.1;
	    
	    if(evEvent.cdDesignator->childForKey("width") != NULL &&
	       evEvent.cdDesignator->childForKey("depth") != NULL &&
	       evEvent.cdDesignator->childForKey("height") != NULL) {
	      fWidth = evEvent.cdDesignator->floatValue("width");
	      fDepth = evEvent.cdDesignator->floatValue("depth");
	      fHeight = evEvent.cdDesignator->floatValue("height");
	    } else {
	      this->info("No dimension (width, depth, height) given for object '" + strObjectName + "', assuming default (0.1, 0.1, 0.1).");
	    }
	    
	    ioNew->setSize(fWidth, fDepth, fHeight);
	    
	    if(evEvent.cdDesignator->childForKey("menu") != NULL) {
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
	      this->info("No menu given for object '" + strObjectName + "'.");
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
