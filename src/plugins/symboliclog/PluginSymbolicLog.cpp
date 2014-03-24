/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


#include <plugins/symboliclog/PluginSymbolicLog.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->setPluginVersion("0.9");
      
      m_prLastFailure = make_pair("", (Node*)NULL);
      
      // Random seed
      srand(time(NULL));
      
      m_ndActive = NULL;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      for(list<Node*>::iterator itNode = m_lstNodes.begin();
	  itNode != m_lstNodes.end();
	  itNode++) {
	Node* ndCurrent = *itNode;
	
	delete ndCurrent;
      }
      
      m_lstNodes.clear();
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Plan node control events
      this->setSubscribedToEvent("begin-context", true);
      this->setSubscribedToEvent("end-context", true);
      
      // Extra information assertion
      this->setSubscribedToEvent("add-designator", true);
      this->setSubscribedToEvent("add-object", true);
      this->setSubscribedToEvent("add-failure", true);
      this->setSubscribedToEvent("catch-failure", true);
      this->setSubscribedToEvent("add-image-from-file", true);
      this->setSubscribedToEvent("equate-designators", true);
      
      // Information supply services
      this->setOffersService("symbolic-plan-tree", true);
      this->setOffersService("symbolic-plan-context", true);
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      
      m_mtxEventsStore.lock();
      resCycle.lstEvents = m_lstEvents;
      m_lstEvents.clear();
      m_mtxEventsStore.unlock();
      
      return resCycle;
    }
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evReturn = defaultEvent();
      
      if(seServiceEvent.siServiceIdentifier == SI_REQUEST) {
	if(seServiceEvent.strServiceName == "symbolic-plan-tree") {
	  // Requested the whole symbolic plan log
	  evReturn.lstNodes = m_lstNodes;
	  
	  evReturn.lstDesignatorIDs = m_lstDesignatorIDs;
	  evReturn.lstEquations = m_lstDesignatorEquations;
	  evReturn.lstEquationTimes = m_lstDesignatorEquationTimes;
	} else if(seServiceEvent.strServiceName == "symbolic-plan-context") {
	  // Requested the current path in the symbolic plan log
	  Node* ndCurrent = this->activeNode();
	  
	  while(ndCurrent) {
	    evReturn.lstNodes.push_back(ndCurrent);
	    ndCurrent = ndCurrent->parent();
	  }
	}
      }
      
      return evReturn;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      //int nID = (evEvent.nContextID == -1 ? (evEvent.cdDesignator ? (int)evEvent.cdDesignator->floatValue("_id") : -1) : evEvent.nContextID);
      
      if(evEvent.strEventName == "begin-context") {
	string strName = evEvent.cdDesignator->stringValue("_name");
	
	Node* ndFormerParent = this->activeNode();
	Node* ndNew = this->addNode(strName, evEvent.nContextID);
	ndNew->setDescription(evEvent.cdDesignator->description());
	
	ndNew->metaInformation()->setValue(string("time-start"), this->getTimeStampStr());
	
	int nDetailLevel = (int)evEvent.cdDesignator->floatValue("_detail-level");
	ndNew->metaInformation()->setValue(string("detail-level"), nDetailLevel);
	
	Event evSymbolicBeginCtx = defaultEvent("symbolic-begin-context");
	evSymbolicBeginCtx.lstNodes.push_back(ndNew);
	this->deployEvent(evSymbolicBeginCtx);
	
	if(ndFormerParent) {
	  Event evSymbolicSetSubcontext = defaultEvent("symbolic-set-subcontext");
	  evSymbolicSetSubcontext.lstNodes.push_back(ndFormerParent);
	  evSymbolicSetSubcontext.lstNodes.push_back(ndNew);
	  this->deployEvent(evSymbolicSetSubcontext);
	}
      } else if(evEvent.strEventName == "end-context") {
	int nID = (int)evEvent.cdDesignator->floatValue("_id");
	
	int nSuccess = (int)evEvent.cdDesignator->floatValue("_success");
	Node *ndCurrent = this->activeNode();
	
	if(ndCurrent) {
	  if(ndCurrent->id() == nID) {
	    stringstream sts;
	    sts << "Received stop context designator for ID " << nID << " (success: " << (nSuccess ? "yes" : "no") << ")";
	    this->info(sts.str());
	    
	    // Set success only if no failures are present (in
	    // which case the success is set to 'false' already)
	    if(!ndCurrent->hasFailures()) {
	      ndCurrent->metaInformation()->setValue(string("success"), nSuccess);
	    }
	    
	    string strTimeEnd = this->getTimeStampStr();
	    ndCurrent->metaInformation()->setValue(string("time-end"), strTimeEnd);
	    
	    Node *ndParent = ndCurrent->parent();
	    this->setNodeAsActive(ndParent);
	    
	    while(ndParent) {
	      if(ndParent->prematurelyEnded()) {
		stringstream sts_id;
		sts_id << ndParent->id();
		this->info("Node ID " + sts_id.str() + " ended prematurely, removing from context stack.");
		
		// Setting the same values for success and end time as
		// for the actually ended node.
		ndParent->metaInformation()->setValue(string("time-end"), strTimeEnd);
		
		// Set success only if no failures are present (in
		// which case the success is set to 'false' already)
		if(!ndParent->hasFailures()) {
		  ndParent->metaInformation()->setValue(string("success"), nSuccess);
		}
		
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
	      string strTimeImage = this->getTimeStampStr();
	      this->activeNode()->addImage(strTopic, strFilepath, strTimeImage);
	      
	      stringstream sts;
	      sts << this->activeNode()->id();
	      this->info("Added image to active node (id " + sts.str() + "): '" + strFilepath + "'");
	      
	      Event evSymbolicAddImage = defaultEvent("symbolic-add-image");
	      evSymbolicAddImage.lstNodes.push_back(this->activeNode());
	      evSymbolicAddImage.cdDesignator = new CDesignator();
	      evSymbolicAddImage.cdDesignator->setType(ACTION);
	      evSymbolicAddImage.cdDesignator->setValue("origin", strTopic);
	      evSymbolicAddImage.cdDesignator->setValue("filename", strFilepath);
	      evSymbolicAddImage.cdDesignator->setValue("time-capture", strTimeImage);
	      this->deployEvent(evSymbolicAddImage);
	    } else {
	      this->warn("No filename given. Will not add unnamed image to active node. The designator was:");
	      evEvent.cdDesignator->printDesignator();
	    }
	  } else {
	    this->warn("No node context available. Cannot add image from file while on top-level.");
	  }
	}
      } else if(evEvent.strEventName == "add-failure") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    // Adding a failure to a node also means to set its success state to 'false'.
	    string strCondition = evEvent.cdDesignator->stringValue("condition");
	    string strTimeFail = this->getTimeStampStr();
	    
	    string strFailureID = this->activeNode()->addFailure(strCondition, strTimeFail);
	    this->replaceStringInPlace(strFailureID, "-", "_");
	    
	    m_prLastFailure = make_pair(strFailureID, this->activeNode());
	    this->activeNode()->setSuccess(false);
	    
	    stringstream sts;
	    sts << this->activeNode()->id();
	    this->info("Added failure '" + m_prLastFailure.first + "' to active node (id " + sts.str() + "): '" + strCondition.c_str() + "'");
	    
	    Event evSymbAddFailure = defaultEvent("symbolic-add-failure");
	    evSymbAddFailure.lstNodes.push_back(this->activeNode());
	    evSymbAddFailure.cdDesignator = new CDesignator();
	    evSymbAddFailure.cdDesignator->setType(ACTION);
	    evSymbAddFailure.cdDesignator->setValue("condition", strCondition);
	    evSymbAddFailure.cdDesignator->setValue("time-failure", strTimeFail);
	    this->deployEvent(evSymbAddFailure);
	  } else {
	    this->warn("No node context available. Cannot add failure while on top-level.");
	  }
	}
      } else if(evEvent.strEventName == "catch-failure") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    if(m_prLastFailure.first != "") {
	      string strID = evEvent.cdDesignator->stringValue("context-id");
	      
	      if(strID != "") {
		int nID;
		sscanf(strID.c_str(), "%d", &nID);
		
		Node* ndRelative = this->activeNode()->relativeWithID(nID);
		if(ndRelative) {
		  ndRelative->catchFailure(m_prLastFailure, this->getTimeStampStr());
		  
		  this->info("Context (ID = " + strID + ") caught failure '" + m_prLastFailure.first + "'");
		  m_prLastFailure = make_pair("", (Node*)NULL);
		} else {
		  this->warn("Relative with ID " + strID + " not found up the chain while catching failure.");
		}
	      } else {
		this->warn("Invalid context ID when catching failure: '" + strID + "'.");
	      }
	    } else {
	      this->warn("Tried to catch failure without one being present.");
	    }
	  } else {
	    this->fail("Cannot catch failures outside of context.");
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
	    
	    // The designator does not yet exist here in case
	    // bDesigExists is false. It is created right after, but
	    // is associated with the current node before already.
	    stringstream sts;
	    sts << this->activeNode()->id();
	    this->info("Added '" + strType + "' designator (addr=" + strMemAddr + ") to active context (id " + sts.str() + "): '" + strUniqueID + "', annotation: '" + strAnnotation + "'");
	    
	    if(!bDesigExists) {
	      CDesignator* cdTemp = new CDesignator((strType == "ACTION" ? ACTION : (strType == "OBJECT" ? OBJECT : LOCATION)),
						    lstDescription);
	      cdTemp->setValue("_id", strUniqueID);
	      
	      if(strAnnotation != "") {
		cdTemp->setValue("_annotation", strAnnotation);
	      }
	      
	      // First, symbolically create the designator
	      Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
	      evLoggedDesignator.cdDesignator = cdTemp;
	      evLoggedDesignator.strAnnotation = strAnnotation;
	      
	      this->deployEvent(evLoggedDesignator);
	      
	      // Second, symbolically add it to the current event
	      Event evAddedDesignator = defaultEvent("symbolic-add-designator");
	      evAddedDesignator.cdDesignator = new CDesignator(cdTemp);
	      evAddedDesignator.lstNodes.push_back(this->activeNode());
	    
	      this->deployEvent(evAddedDesignator);
	    }
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
	      // Check if child designator exists
	      bool bChildDesigExists = (this->getDesignatorID(strMemAddrChild) != "");
	      string strUniqueIDChild = this->getUniqueDesignatorID(strMemAddrChild);
	      if(!bChildDesigExists) {
		this->info("Adding non-existant child-designator during 'equate'");
		
		string strType = evEvent.cdDesignator->stringValue("type-child");
		CKeyValuePair *ckvpDesc = evEvent.cdDesignator->childForKey("description-child");
		list<CKeyValuePair*> lstDescription = ckvpDesc->children();
		
		CDesignator* cdTemp = new CDesignator((strType == "ACTION" ? ACTION : (strType == "OBJECT" ? OBJECT : LOCATION)),
						      lstDescription);
		cdTemp->setValue("_id", strUniqueIDChild);
		
		// First, symbolically create the designator
		Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
		evLoggedDesignator.cdDesignator = cdTemp;
		
		this->deployEvent(evLoggedDesignator);
	      
		// Second, symbolically add it to the current event
		Event evAddedDesignator = defaultEvent("symbolic-add-designator");
		evAddedDesignator.cdDesignator = new CDesignator(cdTemp);
		evAddedDesignator.lstNodes.push_back(this->activeNode());
	    
		this->deployEvent(evAddedDesignator);
	      }
	      
	      // Check if parent designator exists
	      bool bParentDesigExists = (this->getDesignatorID(strMemAddrParent) != "");
	      string strUniqueIDParent = this->getUniqueDesignatorID(strMemAddrParent);
	      if(!bParentDesigExists) {
		this->warn("Adding non-existant parent-designator during 'equate'");
		
		string strType = evEvent.cdDesignator->stringValue("type-parent");
		CKeyValuePair *ckvpDesc = evEvent.cdDesignator->childForKey("description-parent");
		list<CKeyValuePair*> lstDescription = ckvpDesc->children();
		
		CDesignator* cdTemp = new CDesignator((strType == "ACTION" ? ACTION : (strType == "OBJECT" ? OBJECT : LOCATION)),
						      lstDescription);
		cdTemp->setValue("_id", strUniqueIDParent);
		
		// First, symbolically create the designator
		Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
		evLoggedDesignator.cdDesignator = cdTemp;
		
		this->deployEvent(evLoggedDesignator);
		
		// Second, symbolically add it to the current event
		Event evAddedDesignator = defaultEvent("symbolic-add-designator");
		evAddedDesignator.cdDesignator = new CDesignator(cdTemp);
		evAddedDesignator.lstNodes.push_back(this->activeNode());
		
		this->deployEvent(evAddedDesignator);
	      }
	      
	      string strEquationTime = this->equateDesignators(strMemAddrChild, strMemAddrParent);
	      
	      this->info("Equated designators " + strUniqueIDChild + " (successor) and " + strUniqueIDParent + " (parent).");
	      
	      Event evEquateDesigs = defaultEvent("symbolic-equate-designators");
	      evEquateDesigs.cdDesignator = new CDesignator();
	      evEquateDesigs.cdDesignator->setType(ACTION);
	      evEquateDesigs.cdDesignator->setValue("parent-id", strUniqueIDParent);
	      evEquateDesigs.cdDesignator->setValue("child-id", strUniqueIDChild);
	      evEquateDesigs.cdDesignator->setValue("equation-time", strEquationTime);
	      
	      this->deployEvent(evEquateDesigs);
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
	      if(!bDesigExists) { // Object does not yet exist. Add it
				  // symbolically.
		this->info("Adding non-existant object-designator to current context");
		
		CKeyValuePair *ckvpDesc = evEvent.cdDesignator->childForKey("description");
		list<CKeyValuePair*> lstDescription = ckvpDesc->children();
		
		CDesignator* cdTemp = new CDesignator(OBJECT, lstDescription);
		cdTemp->setValue("_id", strUniqueID);
		
		// First, symbolically create the designator
		Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
		evLoggedDesignator.cdDesignator = cdTemp;
		
		this->deployEvent(evLoggedDesignator);
		
		// Second, symbolically add it to the current event
		Event evAddedDesignator = defaultEvent("symbolic-add-designator");
		evAddedDesignator.cdDesignator = new CDesignator(cdTemp);
		evAddedDesignator.lstNodes.push_back(this->activeNode());
		
		this->deployEvent(evAddedDesignator);
	      }
	      
	      ckvpDesc->setValue("__id", strUniqueID);
	      this->activeNode()->addObject(ckvpDesc->children());
	      
	      stringstream sts;
	      sts << this->activeNode()->id();
	      this->info("Added object (" + strUniqueID + ") to active node (id " + sts.str() + ").");
	      
	      // Signal symbolic addition of object
	      Event evSymAddObj = defaultEvent("symbolic-add-object");
	      evSymAddObj.cdDesignator = new CDesignator(OBJECT, ckvpDesc);
	      
	      // Okay, this is pretty hacky. Right now, the CRAM
	      // system does not send object names. That needs to be
	      // fixed. Until then, a dummy name is used here.
	      evSymAddObj.cdDesignator->setValue("name", "object0");//strUniqueID);
	      
	      if(evSymAddObj.cdDesignator->childForKey("pose") != NULL) {
		if(evSymAddObj.cdDesignator->childForKey("pose")->type() == POSESTAMPED) {
		  evSymAddObj.cdDesignator->setValue("pose-stamped", evSymAddObj.cdDesignator->poseStampedValue("pose"));
		} else if(evSymAddObj.cdDesignator->childForKey("pose")->type() == POSE) {
		  evSymAddObj.cdDesignator->setValue("pose", evSymAddObj.cdDesignator->poseStampedValue("pose"));
		}
	      }
	      
	      string strObjName = evSymAddObj.cdDesignator->stringValue("name");
	      
	      CKeyValuePair* ckvpMenu = evSymAddObj.cdDesignator->addChild("menu");
	      CKeyValuePair* ckvpMenuPickObject = ckvpMenu->addChild("PICK-OBJECT");
	      ckvpMenuPickObject->setValue("label", "Pick up object " + strObjName);
	      ckvpMenuPickObject->setValue("parameter", strObjName);
	      
	      this->deployEvent(evSymAddObj);
	    } else {
	      this->warn("No node context available. Cannot add object while on top-level.");
	    }
	  }
	}
      } else {
	this->warn("Unknown event name: '" + evEvent.strEventName + "'");
      }
    }
    
    Node* PLUGIN_CLASS::addNode(string strName, int nContextID) {
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
    
    void PLUGIN_CLASS::setNodeAsActive(Node* ndActive) {
      m_ndActive = ndActive;
      bool bSame = false;
      
      if(m_ndActive && ndActive) {
	bSame = (m_ndActive->id() == ndActive->id());
      }
      
      if(m_ndActive) {
	if(!bSame) {
	  stringstream sts;
	  sts << m_ndActive->id();
	  this->info("Setting context ID " + sts.str() + " as active context");
	}
      } else {
	this->info("Removed active context, returning to top-level");
      }
    }
    
    Node* PLUGIN_CLASS::activeNode() {
      return m_ndActive;
    }
    
    string PLUGIN_CLASS::getDesignatorID(string strMemoryAddress) {
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
    
    string PLUGIN_CLASS::getUniqueDesignatorID(string strMemoryAddress) {
      string strID = this->getDesignatorID(strMemoryAddress);
      
      if(strID == "") {
	strID = this->generateRandomIdentifier("designator_", 14);
	m_lstDesignatorIDs.push_back(make_pair(strMemoryAddress, strID));
      }
      
      return strID;
    }
    
    string PLUGIN_CLASS::generateRandomIdentifier(string strPrefix, unsigned int unLength) {
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
    
    string PLUGIN_CLASS::equateDesignators(string strMAChild, string strMAParent) {
      string strIDChild = this->getUniqueDesignatorID(strMAChild);
      string strIDParent = this->getUniqueDesignatorID(strMAParent);
      
      string strTimeStart = this->getTimeStampStr();
      
      m_lstDesignatorEquations.push_back(make_pair(strIDParent, strIDChild));
      m_lstDesignatorEquationTimes.push_back(make_pair(strIDChild, strTimeStart));
      
      return strTimeStart;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
