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


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->addDependency("imagecapturer");
      this->setPluginVersion("0.93");
      
      m_prLastFailure = std::make_pair("", (Node*)NULL);
      
      // Random seed
      srand(time(NULL));
      
      m_ndActive = NULL;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      for(Node* ndNode : m_lstNodes) {
	delete ndNode;
      }
      
      m_lstNodes.clear();
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Plan node control events
      this->setSubscribedToEvent("begin-context", true);
      this->setSubscribedToEvent("end-context", true);
      
      // Supervisor
      this->setSubscribedToEvent("start-new-experiment", true);
      
      // Extra information assertion
      this->setSubscribedToEvent("add-designator", true);
      this->setSubscribedToEvent("add-object", true);
      this->setSubscribedToEvent("add-failure", true);
      this->setSubscribedToEvent("catch-failure", true);
      this->setSubscribedToEvent("rethrow-failure", true);
      this->setSubscribedToEvent("add-image-from-file", true);
      this->setSubscribedToEvent("equate-designators", true);
      
      // Information supply services
      this->setOffersService("symbolic-plan-tree", true);
      this->setOffersService("symbolic-plan-context", true);
      
      Designator* cdConfig = this->getIndividualConfig();
      std::string strSemanticsDescriptorFile = cdConfig->stringValue("semantics-descriptor-file");
      
      this->setTimeFloatingPointPrecision((int)cdConfig->floatValue("time-precision"));
      
      
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
	  evReturn.lstRootNodes = m_lstRootNodes;
	  
	  evReturn.lstDesignatorIDs = m_lstDesignatorIDs;
	  evReturn.lstEquations = m_lstDesignatorEquations;
	  evReturn.lstEquationTimes = m_lstDesignatorEquationTimes;
	} else if(seServiceEvent.strServiceName == "symbolic-plan-context") {
	  // Requested the current path in the symbolic plan log
	  Node* ndCurrent = this->activeNode();
	  
	  while(ndCurrent) {
	    evReturn.lstNodes.push_back(ndCurrent);
	    evReturn.lstRootNodes = m_lstRootNodes;
	    
	    ndCurrent = ndCurrent->parent();
	  }
	}
      }
      
      return evReturn;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "begin-context") {
	std::string strName = evEvent.cdDesignator->stringValue("_name");
	
	Node* ndFormerParent = this->relativeActiveNode(evEvent);
	Node* ndNew = this->addNode(strName, evEvent.nContextID, ndFormerParent);
	ndNew->setDescription(evEvent.cdDesignator->description());
	
	// This means we're setting an implicit success for this node.
	ndNew->setSuccess(true);
	
	std::string strTimeStart = this->getTimeStampStr();
	if(evEvent.cdDesignator->childForKey("_time-start")) {
	  strTimeStart = this->getTimeStampStr(evEvent.cdDesignator->floatValue("_time-start"));
	}
	
	ndNew->metaInformation()->setValue(std::string("time-start"), strTimeStart);
	
	if(evEvent.cdDesignator->childForKey("_class")) {
	  ndNew->metaInformation()->setValue(std::string("class"), evEvent.cdDesignator->stringValue("_class"));
	  
	  if(evEvent.cdDesignator->childForKey("_classnamespace")) {
	    ndNew->metaInformation()->setValue(std::string("classnamespace"), evEvent.cdDesignator->stringValue("_classnamespace"));
	  }
	}
	
	int nDetailLevel = (int)evEvent.cdDesignator->floatValue("_detail-level");
	ndNew->metaInformation()->setValue(std::string("detail-level"), nDetailLevel);
	
	Event evUpdateExperimentTime = defaultEvent("update-absolute-experiment-start-time");
	evUpdateExperimentTime.lstNodes.push_back(ndNew);
	this->deployEvent(evUpdateExperimentTime);
	
	Event evSymbolicBeginCtx = defaultEvent("symbolic-begin-context");
	evSymbolicBeginCtx.lstNodes.push_back(ndNew);
	this->deployEvent(evSymbolicBeginCtx);
	
	Event evSymbolicSetSubcontext = defaultEvent("symbolic-set-subcontext");
	evSymbolicSetSubcontext.lstNodes.push_back(ndFormerParent);
	evSymbolicSetSubcontext.lstNodes.push_back(ndNew);
	this->deployEvent(evSymbolicSetSubcontext);
      } else if(evEvent.strEventName == "end-context") {
	int nID = (int)evEvent.cdDesignator->floatValue("_id");
	int nSuccess = (int)evEvent.cdDesignator->floatValue("_success");
	Node* ndCurrent = this->relativeActiveNode(evEvent);
	
	if(ndCurrent) {
	  if(ndCurrent->id() == nID || evEvent.cdDesignator->childForKey("_relative_context_id")) {
	    std::stringstream sts;
	    sts << "Received stop context designator for ID " << nID << " (success: " << (nSuccess ? "yes" : "no") << ")";
	    this->info(sts.str());

	    // Set success only if no failures are present (in
	    // which case the success is set to 'false' already)
	    if(!ndCurrent->hasFailures()) {
	      // NOTE(winkler): This would be the right spot to
	      // forward the 'failed' condition towards all underlying
	      // node structures (to signal that this branch failed).
	      ndCurrent->setSuccess(nSuccess);
	    } else {
	      ndCurrent->setSuccess(false);
	    }
	    
	    std::string strTimeEnd = this->getTimeStampStr();
	    if(evEvent.cdDesignator->childForKey("_time-end")) {
	      strTimeEnd = this->getTimeStampStr(evEvent.cdDesignator->floatValue("_time-end"));
	    }
	    ndCurrent->metaInformation()->setValue(std::string("time-end"), strTimeEnd);
	    
	    Node* ndParent = ndCurrent->parent();
	    Node* ndParentLastValid = NULL;
	    this->setNodeAsActive(ndParent);
	    
	    while(ndParent) {
	      ndParentLastValid = ndParent;
	      
	      if(ndParent->prematurelyEnded()) {
		std::stringstream sts_id;
		sts_id << ndParent->id();
		this->info("Node ID " + sts_id.str() + " ended prematurely, removing from context stack.");
		
		// Setting the same values for success and end time as
		// for the actually ended node.
		ndParent->metaInformation()->setValue(std::string("time-end"), strTimeEnd);
		
		// Set success only if no failures are present (in
		// which case the success is set to 'false' already)
		if(!ndParent->hasFailures()) {
		  ndParent->setSuccess(nSuccess);
		} else {
		  ndParent->setSuccess(false);
		}
		
		Event evSymbolicEndCtx = defaultEvent("symbolic-end-context");
		evSymbolicEndCtx.lstNodes.push_back(ndParent);
		this->deployEvent(evSymbolicEndCtx);
		
		ndParent = ndParent->parent();
	      } else {
		break;
	      }
	      
	      this->setNodeAsActive(ndParent);
	    }
	    
	    Event evUpdateExperimentTime;
	    if(ndParentLastValid) {
	      ndParentLastValid->ensureProperty("time-end", strTimeEnd);
	      
	      evUpdateExperimentTime = defaultEvent("update-absolute-experiment-end-time");
	      evUpdateExperimentTime.lstNodes.push_back(ndParentLastValid);
	      this->deployEvent(evUpdateExperimentTime);
	    }
	    
	    evUpdateExperimentTime = defaultEvent("update-absolute-experiment-end-time");
	    evUpdateExperimentTime.lstNodes.push_back(ndCurrent);
	    this->deployEvent(evUpdateExperimentTime);
	    
	    Event evSymbolicEndCtx = defaultEvent("symbolic-end-context");
	    evSymbolicEndCtx.lstNodes.push_back(ndCurrent);
	    this->deployEvent(evSymbolicEndCtx);
	  } else {
	    std::stringstream sts;
	    sts << "Received stop node designator for ID " << nID << " while ID " << ndCurrent->id() << " is active.";
	    this->info(sts.str());
	    
	    std::string strTimeEnd = this->getTimeStampStr();
	    Node* ndEndedPrematurely = NULL;
	    Node* ndSearchTemp = ndCurrent;
	    
	    while(ndSearchTemp) {
	      ndSearchTemp->ensureProperty("time-end", strTimeEnd);
	      
	      Event evUpdateExperimentTime = defaultEvent("update-absolute-experiment-end-time");
	      evUpdateExperimentTime.lstNodes.push_back(ndSearchTemp);
	      this->deployEvent(evUpdateExperimentTime);
	      
	      if(ndSearchTemp->id() == nID) {
		ndEndedPrematurely = ndSearchTemp;
		ndSearchTemp = NULL;
	      } else {
		ndSearchTemp = ndSearchTemp->parent();
	      }
	    }
	    
	    if(ndEndedPrematurely) {
	      // Found the prematurely ended node in this branch
	      std::stringstream sts;
	      sts << "Marking node " << nID << " as prematurely ended.";
	      this->info(sts.str());
	      
	      ndEndedPrematurely->setPrematurelyEnded(true);
	    } else {
	      // Didn't find the prematurely ended node in this branch
	      std::stringstream sts;
	      sts << "The apparently prematurely ended node " << nID << " was not found.";
	      this->warn(sts.str());
	    }
	  }
	} else {
	  std::stringstream sts;
	  sts << "Received stop node designator for ID " << nID << " while in top-level.";
	  this->warn(sts.str());
	}
      } else if(evEvent.strEventName == "add-image-from-file") {
	if(evEvent.cdDesignator) {
	  if(this->activeNode()) {
	    std::string strFilepath = evEvent.cdDesignator->stringValue("filename");
	    std::string strTopic = evEvent.cdDesignator->stringValue("origin");
	    
	    if(strFilepath != "") {
	      std::string strTimeImage = this->getTimeStampStr();
	      
	      Node* ndSubject = this->relativeActiveNode(evEvent);
	      if(ndSubject) {
		ndSubject->addImage(strTopic, strFilepath, strTimeImage);
		
		this->info("Added image to active node (id " + this->str(ndSubject->id()) + "): '" + strFilepath + "'");
		
		Event evSymbolicAddImage = defaultEvent("symbolic-add-image");
		evSymbolicAddImage.lstNodes.push_back(ndSubject);
		evSymbolicAddImage.cdDesignator = new Designator();
		evSymbolicAddImage.cdDesignator->setType(Designator::DesignatorType::ACTION);
		evSymbolicAddImage.cdDesignator->setValue("origin", strTopic);
		evSymbolicAddImage.cdDesignator->setValue("filename", strFilepath);
		evSymbolicAddImage.cdDesignator->setValue("time-capture", strTimeImage);
		evSymbolicAddImage.nOpenRequestID = evEvent.nOpenRequestID;
		this->deployEvent(evSymbolicAddImage);
	      } else {
		this->fail("Cannot add image: Given relative parent node (ID = " + this->str((int)evEvent.cdDesignator->floatValue("_relative_context_id")) + ") does not exist. This is a problem.");
	      }
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
	  Node* ndSubject = this->relativeActiveNode(evEvent);
	  
	  if(ndSubject) {
	    // Adding a failure to a node also means to set its success state to 'false'.
	    ndSubject->setSuccess(false);
	    
	    std::string strCondition = evEvent.cdDesignator->stringValue("condition");
	    std::string strTimeFail = this->getTimeStampStr();
	    
	    std::string strFailureID = ndSubject->addFailure(strCondition, strTimeFail);
	    this->replaceStringInPlace(strFailureID, "-", "_");
	    
	    m_prLastFailure = std::make_pair(strFailureID, ndSubject);
	    
	    this->info("Added failure '" + m_prLastFailure.first + "' to active node (id " + this->str(ndSubject->id()) + "): '" + strCondition.c_str() + "'");
	    
	    Event evSymbAddFailure = defaultEvent("symbolic-add-failure");
	    evSymbAddFailure.lstNodes.push_back(ndSubject);
	    evSymbAddFailure.cdDesignator = new Designator();
	    evSymbAddFailure.cdDesignator->setType(Designator::DesignatorType::ACTION);
	    evSymbAddFailure.cdDesignator->setValue("condition", strCondition);
	    evSymbAddFailure.cdDesignator->setValue("time-fail", strTimeFail);
	    this->deployEvent(evSymbAddFailure);
	  } else {
	    this->warn("No node context available. Cannot add failure while on top-level (this can also mean that the targetted relative node ID does not exist).");
	  }
	}
      } else if(evEvent.strEventName == "catch-failure") {
	if(evEvent.cdDesignator) {
	  Node* ndSubject = this->relativeActiveNode(evEvent);
	  
	  if(ndSubject) {
	    if(m_prLastFailure.first != "") {
	      std::string strID = evEvent.cdDesignator->stringValue("context-id");
	      
	      if(strID != "") {
		int nID;
		sscanf(strID.c_str(), "%d", &nID);
		
		Node* ndRelative = ndSubject->relativeWithID(nID);
		if(ndRelative) {
		  ndRelative->catchFailure(m_prLastFailure.first, m_prLastFailure.second, this->getTimeStampStr());
		  
		  // Associate this failure with its catching node
		  m_mapFailureCatchers[m_prLastFailure.first] = ndRelative;
		  
		  this->info("Context (ID = " + strID + ") caught failure '" + m_prLastFailure.first + "'");
		} else {
		  this->fail("Relative with ID " + strID + " not found up the chain while catching failure.");
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
      } else if(evEvent.strEventName == "rethrow-failure") {
	if(evEvent.cdDesignator) {
	  if(m_prLastFailure.first != "") {
	    if(m_mapFailureCatchers[m_prLastFailure.first]) {
	      m_mapFailureCatchers[m_prLastFailure.first]->removeCaughtFailure(m_prLastFailure.first);
	      m_mapFailureCatchers[m_prLastFailure.first] = NULL;
	      
	      std::string strID = evEvent.cdDesignator->stringValue("context-id");
	      this->info("Context (ID = " + strID + ") rethrew failure '" + m_prLastFailure.first + "'");
	    } else {
	      this->warn("Apparently caught failure '" + m_prLastFailure.first + "' is not in failure catchers map.");
	    }
	  } else {
	    this->warn("Tried to rethrow failure without active failure. This is probably not what you wanted.");
	  }
	}
      } else if(evEvent.strEventName == "add-designator") {
	if(evEvent.cdDesignator) {
	  Node* ndSubject = this->relativeActiveNode(evEvent);
	  
	  if(ndSubject) {
	    std::string strType = evEvent.cdDesignator->stringValue("type");
	    std::string strAnnotation = evEvent.cdDesignator->stringValue("annotation");
	    std::string strMemAddr = evEvent.cdDesignator->stringValue("memory-address");
	    
	    KeyValuePair* ckvpDesc = evEvent.cdDesignator->childForKey("description");
	    
	    std::list<KeyValuePair*> lstDescription = ckvpDesc->children();
	    this->ensureDesignatorPublished(lstDescription, strMemAddr, strType, strAnnotation, true, this->relativeActiveNode(evEvent));
	  } else {
	    this->warn("No node context available. Cannot add designator while on top-level.");
	  }
	}
      } else if(evEvent.strEventName == "equate-designators") {
	if(evEvent.cdDesignator) {
	  std::string strMemAddrChild = evEvent.cdDesignator->stringValue("memory-address-child");
	  std::string strMemAddrParent = evEvent.cdDesignator->stringValue("memory-address-parent");
	  
	  if(strMemAddrChild != "" && strMemAddrParent != "") {
	    Node* ndSubject = this->relativeActiveNode(evEvent);
	    
	    if(ndSubject) {
	      // Check if child designator exists
	      KeyValuePair* ckvpDescChild = evEvent.cdDesignator->childForKey("description-child");
	      
	      if(this->ensureDesignatorPublished(ckvpDescChild->children(), strMemAddrChild, evEvent.cdDesignator->stringValue("type-child"), "", false, this->relativeActiveNode(evEvent))) {
		this->info("Added non-existant child-designator during 'equate'");
	      }
	      
	      // Check if parent designator exists
	      KeyValuePair* ckvpDescParent = evEvent.cdDesignator->childForKey("description-parent");
	      if(this->ensureDesignatorPublished(ckvpDescParent->children(), strMemAddrParent, evEvent.cdDesignator->stringValue("type-parent"), "", false, this->relativeActiveNode(evEvent))) {
		this->warn("Added non-existant parent-designator during 'equate'");
	      }
	      
	      Designator* desigChild = this->makeDesignator(evEvent.cdDesignator->stringValue("type-child"), ckvpDescChild->children());
	      Designator* desigParent = this->makeDesignator(evEvent.cdDesignator->stringValue("type-parent"), ckvpDescParent->children());
	      
	      std::string strEquationTime = this->equateDesignators(strMemAddrChild, desigChild, strMemAddrParent, desigParent);
	      
	      delete desigChild;
	      delete desigParent;
	      
	      std::string strUniqueIDParent = this->getDesignatorID(strMemAddrParent);
	      std::string strUniqueIDChild = this->getDesignatorID(strMemAddrChild);
	      
	      this->info("Equated designators " + strUniqueIDChild + " (successor) and " + strUniqueIDParent + " (parent).");
	      
	      Event evEquateDesigs = defaultEvent("symbolic-equate-designators");
	      evEquateDesigs.cdDesignator = new Designator();
	      evEquateDesigs.cdDesignator->setType(Designator::DesignatorType::ACTION);
	      evEquateDesigs.cdDesignator->setValue("parent-id", strUniqueIDParent);
	      evEquateDesigs.cdDesignator->setValue("child-id", strUniqueIDChild);
	      evEquateDesigs.cdDesignator->setValue("equation-time", strEquationTime);
	      
	      this->deployEvent(evEquateDesigs);
	    }
	  }
	}
      } else if(evEvent.strEventName == "add-object") {
	if(evEvent.cdDesignator) {
	  KeyValuePair* ckvpDesc = evEvent.cdDesignator->childForKey("description");
	  
	  if(ckvpDesc) {
	    Node* ndSubject = this->relativeActiveNode(evEvent);
	    
	    if(ndSubject) {
	      std::string strType = evEvent.cdDesignator->stringValue("type");
	      std::string strMemAddr = evEvent.cdDesignator->stringValue("memory-address");
	      
	      bool bDesigExists = (this->getDesignatorID(strMemAddr) != "");
	      
	      Designator* desigCurrent = this->makeDesignator(strType, ckvpDesc->children());
	      std::string strUniqueID = this->getUniqueDesignatorID(strMemAddr, desigCurrent);
	      delete desigCurrent;
	      
	      if(!bDesigExists) { // Object does not yet exist. Add it symbolically.
		this->info("Adding non-existant object-designator to current context");
		
		KeyValuePair* ckvpDesc = evEvent.cdDesignator->childForKey("description");
		std::list<KeyValuePair*> lstDescription = ckvpDesc->children();
		
		Designator* cdTemp = new Designator(Designator::DesignatorType::OBJECT, lstDescription);
		cdTemp->setValue("_id", strUniqueID);
		
		// First, symbolically create the designator
		Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
		evLoggedDesignator.cdDesignator = cdTemp;
		
		this->deployEvent(evLoggedDesignator);
		
		// Second, symbolically add it to the current event
		Event evAddedDesignator = defaultEvent("symbolic-add-designator");
		evAddedDesignator.cdDesignator = new Designator(cdTemp);
		evAddedDesignator.lstNodes.push_back(ndSubject);
		evAddedDesignator.strAnnotation = evEvent.cdDesignator->stringValue("annotation");
		
		this->deployEvent(evAddedDesignator);
	      }
	      
	      ckvpDesc->setValue("__id", strUniqueID);
	      
	      if(evEvent.cdDesignator->childForKey("class")) {
		ckvpDesc->setValue(std::string("_class"), evEvent.cdDesignator->stringValue("class"));
		
		if(evEvent.cdDesignator->childForKey("classnamespace")) {
		  ckvpDesc->setValue(std::string("_classnamespace"), evEvent.cdDesignator->stringValue("classnamespace"));
		}
	      }
	      
	      if(evEvent.cdDesignator->childForKey("property")) {
		ckvpDesc->setValue(std::string("_property"), evEvent.cdDesignator->stringValue("property"));
	      }
	      
	      if(evEvent.cdDesignator->childForKey("path-to-cad-model")) {
		ckvpDesc->setValue("_pathtocadmodel", evEvent.cdDesignator->stringValue("path-to-cad-model"));
	      }
	      
	      ndSubject->addObject(ckvpDesc->children());
	      this->info("Added object (" + strUniqueID + ") to active node (id " + this->str(ndSubject->id()) + ").");
	      
	      // Signal symbolic addition of object
	      Event evSymAddObj = defaultEvent("symbolic-add-object");
	      evSymAddObj.cdDesignator = new Designator(Designator::DesignatorType::OBJECT, ckvpDesc);
	      
	      // TODO(winkler): Okay, this is pretty hacky. Right now,
	      // the CRAM system does not send object names. That
	      // needs to be fixed. Until then, a dummy name is used
	      // here.
	      evSymAddObj.cdDesignator->setValue("name", "object0");//strUniqueID);
	      
	      if(evSymAddObj.cdDesignator->childForKey("pose") != NULL) {
		if(evSymAddObj.cdDesignator->childForKey("pose")->type() == KeyValuePair::ValueType::POSESTAMPED) {
		  evSymAddObj.cdDesignator->setValue("pose-stamped", evSymAddObj.cdDesignator->poseStampedValue("pose"));
		} else if(evSymAddObj.cdDesignator->childForKey("pose")->type() == KeyValuePair::ValueType::POSE) {
		  evSymAddObj.cdDesignator->setValue("pose", evSymAddObj.cdDesignator->poseStampedValue("pose"));
		}
	      }
	      
	      std::string strObjName = evSymAddObj.cdDesignator->stringValue("name");
	      
	      KeyValuePair* ckvpMenu = evSymAddObj.cdDesignator->addChild("menu");
	      KeyValuePair* ckvpMenuPickObject = ckvpMenu->addChild("PICK-OBJECT");
	      ckvpMenuPickObject->setValue("label", "Pick up object " + strObjName);
	      ckvpMenuPickObject->setValue("parameter", strObjName);
	      
	      this->deployEvent(evSymAddObj);
	    } else {
	      this->warn("No node context available. Cannot add object while on top-level.");
	    }
	  }
	}
      } else if(evEvent.strEventName == "start-new-experiment") {
	this->info("Clearing symbolic log for new experiment.");
	
	m_mapNodeIDs.clear();
	
	for(Node* ndNode : m_lstNodes) {
	  delete ndNode;
	}
	
	m_lstNodes.clear();
	m_lstRootNodes.clear();
	m_ndActive = NULL;
	
	m_lstDesignatorIDs.clear();
	m_lstDesignatorEquations.clear();
	m_lstDesignatorEquationTimes.clear();
	
	m_prLastFailure = std::make_pair("", (Node*)NULL);
	
	srand(time(NULL));
	
	this->info("Ready for new experiment.");
      } else {
	this->warn("Unknown event name: '" + evEvent.strEventName + "'");
      }
    }
    
    Node* PLUGIN_CLASS::addNode(std::string strName, int nContextID, Node* ndParent) {
      Node* ndNew = new Node(strName);
      ndNew->setID(nContextID);
      
      m_mapNodeIDs[nContextID] = ndNew;
      
      if(ndParent == NULL) {
	// No parent mode was manually set. Use the currently active
	// one.
	ndParent = m_ndActive;
      }
      
      bool bSetAsActive = (ndParent == m_ndActive);
      
      if(ndParent == NULL) {
	// Add a new top-level node
	m_lstNodes.push_back(ndNew);
	
	this->info("Adding new top-level context with ID " + this->str(nContextID));
      } else {
	// Add it as a subnode to the current contextual node
	ndParent->addSubnode(ndNew);
	
	this->info("Adding new sub context with ID " + this->str(nContextID));
      }
      
      if(bSetAsActive) {
	this->setNodeAsActive(ndNew);
	this->info("The new context's name is '" + strName + "' (now active)");
      } else {
	this->info("The new context's name is '" + strName + "'");
      }
      
      return ndNew;
    }
    
    void PLUGIN_CLASS::setNodeAsActive(Node* ndActive) {
      bool bSame = false;
      
      if(!m_ndActive && ndActive) {
	m_lstRootNodes.push_back(ndActive);
      }
      
      m_ndActive = ndActive;
      
      if(m_ndActive) {
	bSame = (m_ndActive->id() == ndActive->id());
      }
      
      if(m_ndActive) {
	if(!bSame) {
	  this->info("Setting context ID " + this->str(m_ndActive->id()) + " as active context");	  
	}
	
	// This activates the given node
	Event evActiveNode = defaultEvent("symbolic-node-active");
	evActiveNode.lstNodes.push_back(m_ndActive);
	this->deployEvent(evActiveNode);
      } else {
	this->info("Removed active context, returning to top-level");
	
	// This resets the active node (i.e. no nodes active)
	Event evActiveNode = defaultEvent("symbolic-node-active");
	this->deployEvent(evActiveNode);
      }
    }
    
    Node* PLUGIN_CLASS::activeNode() {
      return m_ndActive;
    }
    
    std::string PLUGIN_CLASS::getDesignatorID(std::string strMemoryAddress) {
      std::string strID = "";
      
      for(std::pair<std::string, std::string> prPair : m_lstDesignatorIDs) {
	if(prPair.first == strMemoryAddress) {
	  strID = prPair.second;
	  break;
	}
      }
      
      return strID;
    }
    
    std::string PLUGIN_CLASS::getDesignatorIDType(Designator* desigCurrent) {
      // Specialize the designator ID type here. For now, just
      // distinguish between action, object, and location designators.
      std::string strType = "designator";
      
      switch(desigCurrent->type()) {
      case Designator::DesignatorType::ACTION:
	strType = "action";
	break;
	
      case Designator::DesignatorType::OBJECT:
	strType = "object";
	break;
	
      case Designator::DesignatorType::LOCATION:
	strType = "location";
	break;

      case Designator::DesignatorType::HUMAN:
	strType = "human";
	break;
	
      default:
	break;
      }
      
      return strType;
    }
    
    std::string PLUGIN_CLASS::getUniqueDesignatorID(std::string strMemoryAddress, Designator* desigCurrent) {
      std::string strID = this->getDesignatorID(strMemoryAddress);
      
      if(strID == "") {
	strID = this->generateRandomIdentifier(this->getDesignatorIDType(desigCurrent) + "_", 14);
	m_lstDesignatorIDs.push_back(std::make_pair(strMemoryAddress, strID));
      }
      
      return strID;
    }
    
    std::string PLUGIN_CLASS::generateRandomIdentifier(std::string strPrefix, unsigned int unLength) {
      std::stringstream sts;
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
    
    std::string PLUGIN_CLASS::equateDesignators(std::string strMAChild, Designator* desigChild, std::string strMAParent, Designator* desigParent) {
      std::string strIDChild = this->getUniqueDesignatorID(strMAChild, desigChild);
      std::string strIDParent = this->getUniqueDesignatorID(strMAParent, desigParent);
      
      std::string strTimeStart = this->getTimeStampStr();
      
      m_lstDesignatorEquations.push_back(std::make_pair(strIDParent, strIDChild));
      m_lstDesignatorEquationTimes.push_back(std::make_pair(strIDChild, strTimeStart));
      
      return strTimeStart;
    }
    
    bool PLUGIN_CLASS::ensureDesignatorPublished(std::list<KeyValuePair*> lstDescription, std::string strMemoryAddress, std::string strType, std::string strAnnotation, bool bAdd, Node* ndRelative) {
      bool bDesigExists = (this->getDesignatorID(strMemoryAddress) != "");
      bool bReturn = false;
      bool bDeleteMe = true;
      
      if(ndRelative == NULL) {
	ndRelative = this->activeNode();
      }
      
      Designator* desigCurrent = this->makeDesignator(strType, lstDescription);
      desigCurrent->setValue("_time_created", this->getTimeStampStr());
      
      std::string strUniqueID = this->getUniqueDesignatorID(strMemoryAddress, desigCurrent);
      
      if(bAdd) {
	ndRelative->addDesignator(strType, desigCurrent->children(), strUniqueID, strAnnotation);
	
	this->info("Added '" + strType + "' designator (addr=" + strMemoryAddress + ") to context (id " + this->str(ndRelative->id()) + "): '" + strUniqueID + "', annotation: '" + strAnnotation + "'");
      }
      
      if(!bDesigExists) {
	desigCurrent->setValue("_id", strUniqueID);
	
	if(strAnnotation != "") {
	  desigCurrent->setValue("_annotation", strAnnotation);
	}
	
	// First, symbolically create the designator
	Event evLoggedDesignator = defaultEvent("symbolic-create-designator");
	evLoggedDesignator.cdDesignator = new Designator(desigCurrent);
	evLoggedDesignator.strAnnotation = strAnnotation;
	
	this->deployEvent(evLoggedDesignator);
	
	// Second, symbolically add it to the current event
	Event evAddedDesignator = defaultEvent("symbolic-add-designator");
	evAddedDesignator.cdDesignator = new Designator(desigCurrent);
	evAddedDesignator.strAnnotation = strAnnotation;
	evAddedDesignator.lstNodes.push_back(ndRelative);
	
	this->deployEvent(evAddedDesignator);
	
	// Thirdly, recurse through possibly nested designators,
	// looking for a ''_designator_memory_address'' field being
	// set.
	this->setNestedDesignatorUniqueIDs(desigCurrent);
	
	bReturn = true;
      }
      
      if(bDeleteMe) {
	delete desigCurrent;
      }
      
      return bReturn;
    }
    
    void PLUGIN_CLASS::setNestedDesignatorUniqueIDs(KeyValuePair* ckvpParent) {
      bool bIsDesignator = false;
      std::string strMemAddr = "";
      
      if(ckvpParent->childForKey("_designator_memory_address")) {
	strMemAddr = ckvpParent->childForKey("_designator_memory_address")->stringValue();
	Designator* desigCurrent = this->makeDesignator("", ckvpParent->children());
	std::string strID = this->getUniqueDesignatorID(strMemAddr, desigCurrent);
	delete desigCurrent;
	
	ckvpParent->setValue("_id", strID);
	bIsDesignator = true;
      }
      
      std::list<KeyValuePair*> lstChildren = ckvpParent->children();
      
      for(KeyValuePair* ckvpChild : lstChildren) {
	this->setNestedDesignatorUniqueIDs(ckvpChild);
      }
      
      if(bIsDesignator) {
	std::string strTypePre = ckvpParent->stringValue("_designator_type");
	std::string strType = (strTypePre == "" ? "OBJECT" : strTypePre);
	
	this->ensureDesignatorPublished(ckvpParent->children(), strMemAddr, strType);
      }
    }
    
    Designator* PLUGIN_CLASS::makeDesignator(Designator::DesignatorType edtType, std::list<KeyValuePair*> lstDescription) {
      return new Designator(edtType, lstDescription);
    }
    
    Designator* PLUGIN_CLASS::makeDesignator(std::string strType, std::list<KeyValuePair*> lstDescription) {
      Designator::DesignatorType edtType = Designator::DesignatorType::UNKNOWN;
      
      if(strType == "ACTION") {
	edtType = Designator::DesignatorType::ACTION;
      } else if(strType == "OBJECT") {
	edtType = Designator::DesignatorType::OBJECT;
      } else if(strType == "LOCATION") {
	edtType = Designator::DesignatorType::LOCATION;
      } else if(strType == "HUMAN") {
	edtType = Designator::DesignatorType::HUMAN;
      }
      
      return this->makeDesignator(edtType, lstDescription);
    }
    
    Node* PLUGIN_CLASS::nodeByID(int nID) {
      if(m_mapNodeIDs.find(nID) != m_mapNodeIDs.end()) {
	return m_mapNodeIDs[nID];
      }
      
      return NULL;
    }
    
    Node* PLUGIN_CLASS::relativeActiveNode(Event evEvent) {
      Node* ndSubject = this->activeNode();
      
      if(evEvent.cdDesignator) {
	if(evEvent.cdDesignator->childForKey("_relative_context_id")) {
	  ndSubject = this->nodeByID(evEvent.cdDesignator->floatValue("_relative_context_id"));
	}
      }
      
      return ndSubject;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
