#include <plugins/symboliclog/PluginSymbolicLog.h>


namespace beliefstate {
  namespace plugins {
    PluginSymbolicLog::PluginSymbolicLog() {
      m_ndActive = NULL;
      this->addDependency("imagecapturer");
    }
    
    PluginSymbolicLog::~PluginSymbolicLog() {
      for(list<Node*>::iterator itNode = m_lstNodes.begin();
	  itNode != m_lstNodes.end();
	  itNode++) {
	Node* ndCurrent = *itNode;
	
	delete ndCurrent;
      }
      
      m_lstNodes.clear();
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
      
      m_mtxEventsStore.lock();
      resCycle.lstEvents = m_lstEvents;
      m_lstEvents.clear();
      m_mtxEventsStore.unlock();
      
      return resCycle;
    }
    
    void PluginSymbolicLog::consumeEvent(Event evEvent) {
      //int nID = (evEvent.nContextID == -1 ? (evEvent.cdDesignator ? (int)evEvent.cdDesignator->floatValue("_id") : -1) : evEvent.nContextID);
      
      switch(evEvent.eiEventIdentifier) {
      case EI_BEGIN_CONTEXT: {
	string strName = evEvent.cdDesignator->stringValue("_name");
	Node* ndNew = this->addNode(strName, evEvent.nContextID);
	
	stringstream stsTimeStart;
	stsTimeStart << this->getTimeStamp();
	ndNew->metaInformation()->setValue(string("time-start"), stsTimeStart.str());
	
	int nDetailLevel = (int)evEvent.cdDesignator->floatValue("_detail-level");
	ndNew->metaInformation()->setValue(string("detail-level"), nDetailLevel);
      } break;
	
      case EI_END_CONTEXT: {
	int nID = (int)evEvent.cdDesignator->floatValue("_id");
	//cout << nID << endl;
	int nSuccess = (int)evEvent.cdDesignator->floatValue("_success");
	Node *ndCurrent = this->activeNode();
	
	if(ndCurrent) {
	  if(ndCurrent->id() == nID) {
	    stringstream sts;
	    sts << "Received stop context designator for ID " << nID << " (success: " << (nSuccess ? "yes" : "no") << ")";
	    this->info(sts.str());
	    
	    ndCurrent->metaInformation()->setValue(string("success"), nSuccess);
	    stringstream stsTimeEnd;
	    stsTimeEnd << this->getTimeStamp();
	    ndCurrent->metaInformation()->setValue(string("time-end"), stsTimeEnd.str());
	    
	    Node *ndParent = ndCurrent->parent();
	    this->setNodeAsActive(ndParent);
	    
	    while(ndParent) {
	      if(ndParent->prematurelyEnded()) {
		ndParent = ndParent->parent();
		this->setNodeAsActive(ndParent);
	      } else {
		this->setNodeAsActive(ndParent);
		break;
	      }
	    }
	  } else {
	    stringstream sts;
	    sts << "Received stop node designator for ID " << nID << " while ID " << ndCurrent->id() << " is active.";
	    this->info(sts.str());
	    
	    Node *ndEndedPrematurely = NULL;
	    Node *ndSearchTemp = ndCurrent->parent();
	    
	    while(ndSearchTemp) {
	      if(ndSearchTemp->id() == nID) {
		ndEndedPrematurely = ndSearchTemp;
		ndSearchTemp = NULL;
	      } else {
		ndSearchTemp = ndSearchTemp->parent();
	      }
	    }
	    
	    if(ndEndedPrematurely) {
	      // Found the prematurely ended node in this branch
	      stringstream sts;
	      sts << "Marking node " << nID << " as prematurely ended.";
	      this->info(sts.str());
	      
	      ndEndedPrematurely->setPrematurelyEnded(true);
	    } else {
	      // Didn't find the prematurely ended node in this branch
	      stringstream sts;
	      sts << "The apparently prematurely ended node " << nID << " was not found. This is probably a problem.";
	      this->warn(sts.str());
	    }
	  }
	} else {
	  stringstream sts;
	  sts << "Received stop node designator for ID " << nID << " while in top-level.";
	  this->warn(sts.str());
	}
      } break;
	
      case EI_ADD_IMAGE_FROM_FILE: {
      } break;
	
      case EI_ADD_FAILURE: {
      } break;
	
      case EI_ADD_DESIGNATOR: {
      } break;
	
      case EI_EQUATE_DESIGNATORS: {
      } break;
	
      case EI_ADD_OBJECT: {
      } break;
	
      case EI_EXTRACT_PLANLOG: {
      } break;
	
      default: {
	this->warn("Unknown event identifier");
      } break;
      }
    }
    
    Node* PluginSymbolicLog::addNode(string strName, int nContextID) {
      Node *ndNew = new Node(strName);
      ndNew->setID(nContextID);
      
      if(m_ndActive == NULL) {
	// Add a new top-level node
	m_lstNodes.push_back(ndNew);
	
	stringstream sts;
	sts << nContextID;
	this->info("Adding new top-level context with ID " + sts.str());
      } else {
	// Add it as a subnode to the current contextual node
	m_ndActive->addSubnode(ndNew);
	
	stringstream sts;
	sts << nContextID;
	this->info("Adding new sub context with ID " + sts.str());
      }
      
      this->info("The new context's name is '" + strName + "'");
      this->setNodeAsActive(ndNew);
      
      return ndNew;
    }
    
    void PluginSymbolicLog::setNodeAsActive(Node* ndActive) {
      m_ndActive = ndActive;
      
      if(m_ndActive) {
	stringstream sts;
	sts << m_ndActive->id();
	this->info("Setting context ID " + sts.str() + " as active context");
      } else {
	this->info("Removed active context, returning to top-level");
      }
    }
    
    Node* PluginSymbolicLog::activeNode() {
      return m_ndActive;
    }
  }
  
  extern "C" plugins::PluginSymbolicLog* createInstance() {
    return new plugins::PluginSymbolicLog();
  }
  
  extern "C" void destroyInstance(plugins::PluginSymbolicLog* icDestroy) {
    delete icDestroy;
  }
}
