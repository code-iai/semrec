#include <plugins/symboliclog/PluginSymbolicLog.h>


namespace beliefstate {
  namespace plugins {
    PluginSymbolicLog::PluginSymbolicLog() {
      m_ndActive = NULL;
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
      this->setSubscribedToEvent("begin-context", true);
      this->setSubscribedToEvent("end-context", true);
      
      // Extra information assertion
      this->setSubscribedToEvent("add-designator", true);
      this->setSubscribedToEvent("add-object", true);
      this->setSubscribedToEvent("add-failure", true);
      this->setSubscribedToEvent("add-image-from-file", true);
      this->setSubscribedToEvent("equate-designators", true);
      
      // Information supply services
      this->setOffersService("symbolic-plan-tree", true);
      
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
    
    Event PluginSymbolicLog::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evReturn = defaultEvent();
      
      if(seServiceEvent.siServiceIdentifier == SI_REQUEST) {
	if(seServiceEvent.strServiceName == "symbolic-plan-tree") {
	  evReturn.lstNodes = m_lstNodes;
	  
	  evReturn.lstDesignatorIDs = m_lstDesignatorIDs;
	  evReturn.lstEquations = m_lstDesignatorEquations;
	  evReturn.lstEquationTimes = m_lstDesignatorEquationTimes;
	}
      }
      
      return evReturn;
    }
    
    void PluginSymbolicLog::consumeEvent(Event evEvent) {
      //int nID = (evEvent.nContextID == -1 ? (evEvent.cdDesignator ? (int)evEvent.cdDesignator->floatValue("_id") : -1) : evEvent.nContextID);
      
      if(evEvent.strEventName == "begin-context") {
	string strName = evEvent.cdDesignator->stringValue("_name");
	Node* ndNew = this->addNode(strName, evEvent.nContextID);
	
	stringstream stsTimeStart;
	stsTimeStart << this->getTimeStamp();
	ndNew->metaInformation()->setValue(string("time-start"), stsTimeStart.str());
	
	int nDetailLevel = (int)evEvent.cdDesignator->floatValue("_detail-level");
	ndNew->metaInformation()->setValue(string("detail-level"), nDetailLevel);
	
	Event evSymbolicBeginCtx = defaultEvent("symbolic-begin-context");
	evSymbolicBeginCtx.lstNodes.push_back(ndNew);
	this->deployEvent(evSymbolicBeginCtx);
      } else if(evEvent.strEventName == "end-context") {
	int nID = (int)evEvent.cdDesignator->floatValue("_id");
	
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
	    
	    Event evSymbolicEndCtx = defaultEvent("symbolic-end-context");
	    evSymbolicEndCtx.lstNodes.push_back(ndCurrent);
	    this->deployEvent(evSymbolicEndCtx);
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
      } else if(evEvent.strEventName == "add-image-from-file") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    string strFilepath = evEvent.cdDesignator->stringValue("filename");
	    string strTopic = evEvent.cdDesignator->stringValue("origin");
	    
	    if(strFilepath != "") {
	      this->activeNode()->addImage(strTopic, strFilepath);
	      
	      stringstream sts;
	      sts << this->activeNode()->id();
	      this->info("Added image to active node (id " + sts.str() + "): '" + strFilepath + "'");
	    } else {
	      this->warn("No filename given. Will not add unnamed image to active node. The designator was:");
	      evEvent.cdDesignator->printDesignator();
	    }
	  } else {
	    this->warn("No node context available. Cannot add image from file while on top-level.");
	  }
	}
	this->warn("Adding images from file is not yet implemented!");
      } else if(evEvent.strEventName == "add-failure") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    // Adding a failure to a node also means to set its success state to 'false'.
	    string strCondition = evEvent.cdDesignator->stringValue("condition");
	    
	    stringstream stsTimeFail;
	    stsTimeFail << this->getTimeStamp();
	    
	    this->activeNode()->addFailure(strCondition, stsTimeFail.str());
	    this->activeNode()->setSuccess(false);
	    
	    stringstream sts;
	    sts << this->activeNode()->id();
	    this->info("Added failure to active node (id " + sts.str() + "): '" + strCondition.c_str() + "'");
	  } else {
	    this->warn("No node context available. Cannot add failure while on top-level.");
	  }
	}
      } else if(evEvent.strEventName == "add-designator") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    string strType = evEvent.cdDesignator->stringValue("type");
	    string strAnnotation = evEvent.cdDesignator->stringValue("annotation");
	    string strMemAddr = evEvent.cdDesignator->stringValue("memory-address");
	    
	    CKeyValuePair *ckvpDesc = evEvent.cdDesignator->childForKey("description");
	    list<CKeyValuePair*> lstDescription = ckvpDesc->children();
	    
	    bool bDesigExists = (this->getDesignatorID(strMemAddr) != "");
	    string strUniqueID = this->getUniqueDesignatorID(strMemAddr);
	    
	    this->activeNode()->addDesignator(strType, lstDescription, strUniqueID, strAnnotation);
	    
	    stringstream sts;
	    sts << this->activeNode()->id();
	    this->info("Added '" + strType + "' designator (addr=" + strMemAddr + ") to active context (id " + sts.str() + "): '" + strUniqueID + "'");
	    // desigResponse->setValue("id", strUniqueID);
	    // desigResponse->setValue(string("is-new"), (bDesigExists ? 0.0 : 1.0));
	    
	    //bReturnvalue = true;
	  } else {
	    this->warn("No node context available. Cannot add designator while on top-level.");
	  }
	}
      } else if(evEvent.strEventName == "equate-designators") {
	if(evEvent.cdDesignator) {
	  string strMemAddrChild = evEvent.cdDesignator->stringValue("memory-address-child");
	  string strMemAddrParent = evEvent.cdDesignator->stringValue("memory-address-parent");
	  
	  if(strMemAddrChild != "" && strMemAddrParent != "") {
	    if(this->activeNode()) {
	      this->equateDesignators(strMemAddrChild, strMemAddrParent);
	    }
	  }
	}
      } else if(evEvent.strEventName == "add-object") {
	if(evEvent.cdDesignator) {
	  CKeyValuePair *ckvpDesc = evEvent.cdDesignator->childForKey("description");
	  
	  if(ckvpDesc) {
	    if(this->activeNode()) {
	      string strType = evEvent.cdDesignator->stringValue("type");
	      string strMemAddr = evEvent.cdDesignator->stringValue("memory-address");
	      
	      bool bDesigExists = (this->getDesignatorID(strMemAddr) != "");
	      string strUniqueID = this->getUniqueDesignatorID(strMemAddr);
	      
	      ckvpDesc->setValue("__id", strUniqueID);
	      this->activeNode()->addObject(ckvpDesc->children());
	      
	      stringstream sts;
	      sts << this->activeNode()->id();
	      this->info("Added object (" + strUniqueID + ") to active node (id " + sts.str() + ").");
	    } else {
	      this->warn("No node context available. Cannot add object while on top-level.");
	    }
	  }
	}
      } else {
	this->warn("Unknown event name: '" + evEvent.strEventName + "'");
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
    
    string PluginSymbolicLog::getDesignatorID(string strMemoryAddress) {
      string strID = "";
      
      for(list< pair<string, string> >::iterator itPair = m_lstDesignatorIDs.begin();
	  itPair != m_lstDesignatorIDs.end();
	  itPair++) {
	pair<string, string> prPair = *itPair;
	
	if(prPair.first == strMemoryAddress) {
	  strID = prPair.second;
	  break;
	}
      }
      
      return strID;
    }
    
    string PluginSymbolicLog::getUniqueDesignatorID(string strMemoryAddress) {
      string strID = this->getDesignatorID(strMemoryAddress);
      
      if(strID == "") {
	strID = this->generateRandomIdentifier("designator_", 14);
	m_lstDesignatorIDs.push_back(make_pair(strMemoryAddress, strID));
      }
      
      return strID;
    }
    
    string PluginSymbolicLog::generateRandomIdentifier(string strPrefix, unsigned int unLength) {
      stringstream sts;
      sts << strPrefix;
      
      for(unsigned int unI = 0; unI < unLength; unI++) {
	int nRandom;
	do {
	  nRandom = rand() % 122 + 48;
	} while(nRandom < 48 ||
		(nRandom > 57 && nRandom < 65) ||
		(nRandom > 90 && nRandom < 97) ||
		nRandom > 122);
	
	char cRandom = (char)nRandom;
	sts << cRandom;
      }
      
      return sts.str();
    }
    
    void PluginSymbolicLog::equateDesignators(string strMAChild, string strMAParent) {
      string strIDChild = this->getUniqueDesignatorID(strMAChild);
      string strIDParent = this->getUniqueDesignatorID(strMAParent);
      
      stringstream stsTimeEquate;
      stsTimeEquate << this->getTimeStamp();
      
      m_lstDesignatorEquations.push_back(make_pair(strIDParent, strIDChild));
      m_lstDesignatorEquationTimes.push_back(make_pair(strIDChild, stsTimeEquate.str()));
    }
  }
  
  extern "C" plugins::PluginSymbolicLog* createInstance() {
    return new plugins::PluginSymbolicLog();
  }
  
  extern "C" void destroyInstance(plugins::PluginSymbolicLog* icDestroy) {
    delete icDestroy;
  }
}
